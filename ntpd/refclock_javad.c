/*
 * Copyright (c) 2018
 *      The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#if defined(REFCLOCK) && defined(CLOCK_JAVAD) && defined(HAVE_PPSAPI)

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_refclock.h"
#include "ntp_unixtime.h"
#include "ntp_stdlib.h"

#include <stdio.h>
#include <ctype.h>

#ifdef HAVE_PPSAPI
# include "ppsapi_timepps.h"
#endif

#ifdef WORDS_BIGENDIAN
#define getshort(s) ((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))
#define putshort(s) ((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))
#else
#define getshort(s) ((u_short)(s))
#define putshort(s) ((u_short)(s))
#endif

/*
 * XXX
 * This driver supports Javad GREIS Satellite Receivers. 
 *
 * It requires the ppsclock line discipline or streams module
 * described in the Line Disciplines and Streams Drivers page. It
 * also requires a gadget box and 1-PPS level converter, such as
 * described in the Pulse-per-second (PPS) Signal Interfacing page.
 */

/*
 * GPS Definitions
 */
#define	DEVICE		"/dev/gps%d"	/* device name and unit */
#define	SPEED232	B115200		/* baud */

/*
 * Radio interface parameters
 */
#define	PRECISION	(-18)		/* precision assumed (about 4 us) */
#define	REFID		"GPS\0"		/* reference id */
#define	DESCRIPTION	"Javad Satellite Receiver"
#define	DEFFUDGETIME	0		/* default fudge time (ms) */

/* Unix timestamp for the GPS epoch: January 6, 1980 */
#define GPS_EPOCH 315964800

/* Double short to unsigned int */
#define DS2UI(p) ((getshort((p)[1]) << 16) | getshort((p)[0]))

/* Double short to signed int */
#define DS2I(p) ((getshort((p)[1]) << 16) | getshort((p)[0]))

/* One week's worth of seconds */
#define WEEKSECS (7 * 24 * 60 * 60)

/*
 * Jupiter unit control structure.
 */
struct instance {
	struct peer *peer;		/* peer */
	u_int  pollcnt;			/* poll message counter */
	u_int  polled;			/* Hand in a time sample? */
#ifdef HAVE_PPSAPI
	pps_params_t pps_params;	/* pps parameters */
	pps_info_t pps_info;		/* last pps data */
	pps_handle_t pps_handle;	/* pps handle */
	u_int assert;			/* pps edge to use */
	u_int hardpps;			/* enable kernel mode */
	struct timespec ts;		/* last timestamp */
#endif
	l_fp limit;
	u_int gpos_gweek;		/* Current GPOS GPS week number */
	u_int gpos_sweek;		/* Current GPOS GPS seconds into week */
	u_int gweek;			/* current GPS week number */
	u_int32 lastsweek;		/* last seconds into GPS week */
	time_t timecode;		/* current ntp timecode */
	u_int32 stime;			/* used to detect firmware bug */
	int wantid;			/* don't reconfig on channel id msg */
	u_int  moving;			/* mobile platform? */
	u_char sloppyclockflag;		/* fudge flags */
	char ibuf[512];			/* local input buffer */
	int ssize;			/* space used in sbuf */
};

/*
 * Function prototypes
 */
static void javad_canmsg(struct instance *, u_int);
static u_short javad_cksum(u_short *, u_int);
static int javad_config(struct instance *);
static void javad_debug(struct peer *, const char *, const char *, ...)
    __attribute__ ((format (printf, 3, 4)));
static const char *javad_parse_t(struct instance *, u_short *);
static const char *javad_parse_gpos(struct instance *, u_short *);
static void javad_platform(struct instance *, u_int);
static void javad_poll(int, struct peer *);
static void javad_control(int, const struct refclockstat *,
    struct refclockstat *, struct peer *);
#ifdef HAVE_PPSAPI
static int javad_ppsapi(struct instance *);
static int javad_pps(struct instance *);
#endif /* HAVE_PPSAPI */
static int javad_recv(struct instance *);
static void javad_receive(struct recvbuf *rbufp);
static void javad_reqmsg(struct instance *, u_int, u_int);
static void javad_shutdown(int, struct peer *);

static int javad_start(int, struct peer *);

static const char *javad_send(struct instance *, const char *);
#ifdef notdef
static const char *javad_recvempty(struct instance *);
#endif

/*
 * Transfer vector
 */
struct refclock refclock_javad = {
	javad_start,		/* start up driver */
	javad_shutdown,		/* shut down driver */
	javad_poll,		/* transmit poll message */
	javad_control,		/* (clock control) */
	noentry,		/* (clock init) */
	noentry,		/* (clock buginfo) */
	NOFLAGS			/* not used */
};

/*
 * javad_start - open the devices and initialize data for processing
 */
static int
javad_start(int unit, struct peer *peer)
{
	struct refclockproc *pp;
	struct instance *instance;
	int fd;
	char gpsdev[20];

	/*
	 * Open serial port
	 */
	snprintf(gpsdev, sizeof(gpsdev), DEVICE, unit);
	fd = refclock_open(gpsdev, SPEED232, LDISC_RAW);
	if (fd <= 0) {
		javad_debug(peer, __func__, "open %s: %m", gpsdev);
		return (0);
	}

	/* Allocate unit structure */
	instance = emalloc_zero(sizeof(*instance));
	instance->peer = peer;
	pp = peer->procptr;
	pp->io.clock_recv = javad_receive;
	pp->io.srcclock = peer;
	pp->io.datalen = 0;
	pp->io.fd = fd;
	if (!io_addclock(&pp->io)) {
		close(fd);
		pp->io.fd = -1;
		free(instance);
		return (0);
	}
	pp->unitptr = instance;

	/*
	 * Initialize miscellaneous variables
	 */
	peer->precision = PRECISION;
	pp->clockdesc = DESCRIPTION;
	memcpy((char *)&pp->refid, REFID, 4);

#ifdef HAVE_PPSAPI
	instance->assert = 1;
	instance->hardpps = 0;
	/*
	 * Start the PPSAPI interface if it is there. Default to use
	 * the assert edge and do not enable the kernel hardpps.
	 */
	if (time_pps_create(fd, &instance->pps_handle) < 0) {
		instance->pps_handle = 0;
		msyslog(LOG_ERR,
			"refclock_javad: time_pps_create failed: %m");
	}
	else if (!javad_ppsapi(instance))
		goto clean_up;
#endif /* HAVE_PPSAPI */

	/* Ensure the receiver is properly configured */
	if (!javad_config(instance))
		goto clean_up;

	return (1);

clean_up:
	javad_shutdown(unit, peer);
	pp->unitptr = 0;
	return (0);
}

/*
 * javad_shutdown - shut down the clock
 */
static void
javad_shutdown(int unit, struct peer *peer)
{
	struct instance *instance;
	struct refclockproc *pp;
	// const char *errmsg;

	pp = peer->procptr;
	instance = pp->unitptr;
	if (!instance)
		return;

#ifdef HAVE_PPSAPI
	if (instance->pps_handle) {
		time_pps_destroy(instance->pps_handle);
		instance->pps_handle = 0;
	}
#endif /* HAVE_PPSAPI */

	if (pp->io.fd != -1)
		io_closeclock(&pp->io);
	free(instance);
}

/*
 * javad_config - Configure the receiver
 */
static int
javad_config(struct instance *instance)
{
	const char *errmsg;

	javad_debug(instance->peer, __func__, "init receiver");

	/*
	 * Initialize the unit variables
	 */
	instance->sloppyclockflag = instance->peer->procptr->sloppyclockflag;
#ifdef notdef
	instance->moving = !!(instance->sloppyclockflag & CLK_FLAG2);
	if (instance->moving)
		javad_debug(instance->peer, __func__, "mobile platform");
#endif

	instance->pollcnt = 2;
	instance->polled = 0;
	instance->gpos_gweek = 0;
	instance->gpos_sweek = 0;
	instance->gweek = 0;
	instance->lastsweek = 2 * WEEKSECS;
	instance->timecode = 0;
	instance->stime = 0;
	instance->ssize = 0;

	/* Stop outputting all messages */
	errmsg = javad_send(instance, "dm,/cur/term");

	/* Don't output NEMA messages when UTC time is unavailable */
	if (errmsg == NULL)
		errmsg = javad_send(instance, "set,/par/nmea/notime,off");

	/* RMC once a second */
	if (errmsg == NULL)
		errmsg = javad_send(instance, "em,,nmea/RMC:1");

	// print,/par/pos/hold/alt:on		// on
	// print,/par/nmea/notime:on		// off
	// print,/par/pos/clk/fixpos:on		// on
	// print,/par/dev/pps/a/out:on		// on
	// print,/par/dev/pps/a/time:on		// utc
	// print,/par/dev/pps/a/tied:on		// on
	// print,/par/dev/pps/a/per/ms:on	// 1000
	// print,/par/dev/pps/a/edge:on		// rise
	// print,/par/dev/pps/a/time:on		// utc



	/* How'd we do? */
	if (errmsg != NULL) {
		msyslog(LOG_ERR, "%s: init failed: %s", __func__, errmsg);
		return (0);
	}

	return (1);
}

#ifdef HAVE_PPSAPI
/*
 * Initialize PPSAPI
 */
int
javad_ppsapi(
	struct instance *instance	/* unit structure pointer */
	)
{
	int capability;

	if (time_pps_getcap(instance->pps_handle, &capability) < 0) {
		msyslog(LOG_ERR,
		    "refclock_javad: time_pps_getcap failed: %m");
		return (0);
	}
	memset(&instance->pps_params, 0, sizeof(pps_params_t));
	if (!instance->assert)
		instance->pps_params.mode = capability & PPS_CAPTURECLEAR;
	else
		instance->pps_params.mode = capability & PPS_CAPTUREASSERT;
	if (!(instance->pps_params.mode & (PPS_CAPTUREASSERT | PPS_CAPTURECLEAR))) {
		msyslog(LOG_ERR,
		    "refclock_javad: invalid capture edge %d",
		    instance->assert);
		return (0);
	}
	instance->pps_params.mode |= PPS_TSFMT_TSPEC;
	if (time_pps_setparams(instance->pps_handle, &instance->pps_params) < 0) {
		msyslog(LOG_ERR,
		    "refclock_javad: time_pps_setparams failed: %m");
		return (0);
	}
	if (instance->hardpps) {
		if (time_pps_kcbind(instance->pps_handle, PPS_KC_HARDPPS,
				    instance->pps_params.mode & ~PPS_TSFMT_TSPEC,
				    PPS_TSFMT_TSPEC) < 0) {
			msyslog(LOG_ERR,
			    "refclock_javad: time_pps_kcbind failed: %m");
			return (0);
		}
		hardpps_enable = 1;
	}
/*	instance->peer->precision = PPS_PRECISION; */

#if DEBUG
	if (debug) {
		time_pps_getparams(instance->pps_handle, &instance->pps_params);
		javad_debug(instance->peer, __func__,
			"pps capability 0x%x version %d mode 0x%x kern %d",
			capability, instance->pps_params.api_version,
			instance->pps_params.mode, instance->hardpps);
	}
#endif

	return (1);
}

/*
 * Get PPSAPI timestamps.
 *
 * Return 0 on failure and 1 on success.
 */
static int
javad_pps(struct instance *instance)
{
	pps_info_t pps_info;
	struct timespec timeout, ts;
	double dtemp;
	l_fp tstmp;

	/*
	 * Convert the timespec nanoseconds field to ntp l_fp units.
	 */ 
	if (instance->pps_handle == 0)
		return 1;
	timeout.tv_sec = 0;
	timeout.tv_nsec = 0;
	memcpy(&pps_info, &instance->pps_info, sizeof(pps_info_t));
	if (time_pps_fetch(instance->pps_handle, PPS_TSFMT_TSPEC, &instance->pps_info,
	    &timeout) < 0)
		return 1;
	if (instance->pps_params.mode & PPS_CAPTUREASSERT) {
		if (pps_info.assert_sequence ==
		    instance->pps_info.assert_sequence)
			return 1;
		ts = instance->pps_info.assert_timestamp;
	} else if (instance->pps_params.mode & PPS_CAPTURECLEAR) {
		if (pps_info.clear_sequence ==
		    instance->pps_info.clear_sequence)
			return 1;
		ts = instance->pps_info.clear_timestamp;
	} else {
		return 1;
	}
	if ((instance->ts.tv_sec == ts.tv_sec) && (instance->ts.tv_nsec == ts.tv_nsec))
		return 1;
	instance->ts = ts;

	tstmp.l_ui = (u_int32)ts.tv_sec + JAN_1970;
	dtemp = ts.tv_nsec * FRAC / 1e9;
	tstmp.l_uf = (u_int32)dtemp;
	instance->peer->procptr->lastrec = tstmp;
	return 0;
}
#endif /* HAVE_PPSAPI */

/*
 * javad_poll - javad watchdog routine
 */
static void
javad_poll(int unit, struct peer *peer)
{
#ifdef notdef
	struct instance *instance;
	struct refclockproc *pp;

	pp = peer->procptr;
	instance = pp->unitptr;

	/*
	 * You don't need to poll this clock.  It puts out timecodes
	 * once per second.  If asked for a timestamp, take note.
	 * The next time a timecode comes in, it will be fed back.
	 */

	/*
	 * If we haven't had a response in a while, reset the receiver.
	 */
	if (instance->pollcnt > 0) {
		instance->pollcnt--;
	} else {
		refclock_report(peer, CEVNT_TIMEOUT);

		/* Request the receiver id to trigger a reconfig */
		// javad_reqonemsg(instance, JAVAD_O_ID);
		instance->wantid = 0;
	}

	/*
	 * polled every 64 seconds. Ask javad_receive to hand in
	 * a timestamp.
	 */
	instance->polled = 1;
	pp->polls++;
#endif
}

/*
 * javad_control - fudge control
 */
static void
javad_control(
	int unit,		/* unit (not used) */
	const struct refclockstat *in, /* input parameters (not used) */
	struct refclockstat *out, /* output parameters (not used) */
	struct peer *peer	/* peer structure pointer */
	)
{
	struct refclockproc *pp;
	struct instance *instance;
	u_char sloppyclockflag;

	pp = peer->procptr;
	instance = pp->unitptr;

	DTOLFP(pp->fudgetime2, &instance->limit);
	/* Force positive value. */
	if (L_ISNEG(&instance->limit))
		L_NEG(&instance->limit);

#ifdef HAVE_PPSAPI
	instance->assert = !(pp->sloppyclockflag & CLK_FLAG3);
	javad_ppsapi(instance);
#endif /* HAVE_PPSAPI */

	sloppyclockflag = instance->sloppyclockflag;
	instance->sloppyclockflag = pp->sloppyclockflag;
	if ((instance->sloppyclockflag & CLK_FLAG2) !=
	    (sloppyclockflag & CLK_FLAG2)) {
		javad_debug(peer, __func__,
		    "mode switch: reset receiver");
		javad_config(instance);
		return;
	}
}

/*
 * javad_receive - receive gps data
 * Gag me!
 */
static void
javad_receive(struct recvbuf *rbufp)
{
	int len, i;
	char ch;
	// int cc, size;
	// int ppsret;
	// time_t last_timecode;
	// u_int32 laststime;
	char *cp, *eol;
	// int numcrnl;
	// u_short *sp;
	// struct jid *ip;
	// struct jheader *hp;
	struct peer *peer;
	struct refclockproc *pp;
	struct instance *instance;
	// l_fp tstamp;

	/* Initialize pointers and read the timecode and timestamp */
	peer = rbufp->recv_peer;
	pp = peer->procptr;
	instance = pp->unitptr;

	cp = (char *)rbufp->recv_buffer;
	len = rbufp->recv_length;

	/* This shouldn't happen */
	if (len > sizeof(instance->ibuf) - (instance->ssize - 1))
		len = sizeof(instance->ibuf) - (instance->ssize - 1);

	/* Append to input buffer */
	strlcpy(instance->ibuf + instance->ssize, cp, len + 1);
	instance->ssize += len;

	/* See if have a complete message */
	cp = instance->ibuf;
	len = instance->ssize;
	eol = NULL;
	while (len >= 2) {
		if (*cp++ == '\r' && *cp == '\n') {
			eol = cp + 1;
			break;
		}
		--len;
	}

	if (eol == NULL)
		return;

	fprintf(stderr, "javad_receive: %d \"", instance->ssize);

	cp = instance->ibuf;
	len = eol - cp;
	for (i = len; i > 0; --i) {
		ch = *cp++;
		if (ch == '\r')
			fprintf(stderr, "\\r");
		else if (ch == '\n')
			fprintf(stderr, "\\n");
		else if (!isprint(ch))
			fprintf(stderr, "\\0%o", ((int)ch) & 0xff);
		else
			fprintf(stderr, "%c", ch);
	}
	fprintf(stderr, "\"\n");


	/* Housekeeping */
	if (instance->ssize == len) {
		instance->ibuf[0] = '\0';
		instance->ssize = 0;
	} else if (instance->ssize > len) {
		/* Shift leftovers */
		memcpy(instance->ibuf, (char *)instance->ibuf + len,
		    instance->ssize + 1);
		instance->ssize -= len;
	} else {
		fprintf(stderr, "jupiter_recv: ssize < len! (%d < %d)\n",
		    instance->ssize, len);
		abort();
	}
}


#ifdef notdef
static const char *
javad_parse_t(struct instance *instance, u_short *sp)
{
	struct tm *tm;
	char *cp;
	struct jpulse *jp;
	u_int32 sweek;
	time_t last_timecode;
	u_short flags;

	jp = (struct jpulse *)sp;

	/* The timecode is presented as seconds into the current GPS week */
	sweek = DS2UI(jp->sweek) % WEEKSECS;

	/*
	 * If we don't know the current GPS week, calculate it from the
	 * current time. (It's too bad they didn't include this
	 * important value in the pulse message). We'd like to pick it
	 * up from one of the other messages like gpos or chan but they
	 * don't appear to be synchronous with time keeping and changes
	 * too soon (something like 10 seconds before the new GPS
	 * week).
	 *
	 * If we already know the current GPS week, increment it when
	 * we wrap into a new week.
	 */
	if (instance->gweek == 0) {
		if (!instance->gpos_gweek) {
			return ("javad_parse_t: Unknown gweek");
		}

		instance->gweek = instance->gpos_gweek;

		/*
		 * Fix warps. GPOS has GPS time and PULSE has UTC.
		 * Plus, GPOS need not be completely in synch with
		 * the PPS signal.
		 */
		if (instance->gpos_sweek >= sweek) {
			if ((instance->gpos_sweek - sweek) > WEEKSECS / 2)
				++instance->gweek;
		}
		else {
			if ((sweek - instance->gpos_sweek) > WEEKSECS / 2)
				--instance->gweek;
		}
	}
	else if (sweek == 0 && instance->lastsweek == WEEKSECS - 1) {
		++instance->gweek;
		javad_debug(instance->peer, __func__,
		    "NEW gps week %u", instance->gweek);
	}

	/*
	 * See if the sweek stayed the same (this happens when there is
	 * no pps pulse).
	 *
	 * Otherwise, look for time warps:
	 *
	 *   - we have stored at least one lastsweek and
	 *   - the sweek didn't increase by one and
	 *   - we didn't wrap to a new GPS week
	 *
	 * Then we warped.
	 */
	if (instance->lastsweek == sweek)
		javad_debug(instance->peer, __func__,
		    "gps sweek not incrementing (%d)",
		    sweek);
	else if (instance->lastsweek != 2 * WEEKSECS &&
	    instance->lastsweek + 1 != sweek &&
	    !(sweek == 0 && instance->lastsweek == WEEKSECS - 1))
		javad_debug(instance->peer, __func__,
		    "gps sweek jumped (was %d, now %d)",
		    instance->lastsweek, sweek);
	instance->lastsweek = sweek;

	/* This timecode describes next pulse */
	last_timecode = instance->timecode;
	instance->timecode =
	    GPS_EPOCH + (instance->gweek * WEEKSECS) + sweek;

	if (last_timecode == 0)
		/* XXX debugging */
		javad_debug(instance->peer, __func__,
		    "UTC <none> (gweek/sweek %u/%u)",
		    instance->gweek, sweek);
	else {
		/* XXX debugging */
		tm = gmtime(&last_timecode);
		cp = asctime(tm);

		javad_debug(instance->peer, __func__,
		    "UTC %.24s (gweek/sweek %u/%u)",
		    cp, instance->gweek, sweek);

		/* Billboard last_timecode (which is now the current time) */
		instance->peer->procptr->year   = tm->tm_year + 1900;
		instance->peer->procptr->day    = tm->tm_yday + 1;
		instance->peer->procptr->hour   = tm->tm_hour;
		instance->peer->procptr->minute = tm->tm_min;
		instance->peer->procptr->second = tm->tm_sec;
	}

	flags = getshort(jp->flags);

	/* Toss if not designated "valid" by the gps */
	if ((flags & JAVAD_O_PULSE_VALID) == 0) {
		refclock_report(instance->peer, CEVNT_BADTIME);
		return ("time mark not valid");
	}

	/* We better be sync'ed to UTC... */
	if ((flags & JAVAD_O_PULSE_UTC) == 0) {
		refclock_report(instance->peer, CEVNT_BADTIME);
		return ("time mark not sync'ed to UTC");
	}

	return (NULL);
}

static const char *
javad_parse_gpos(struct instance *instance, u_short *sp)
{
	struct jgpos *jg;
	time_t t;
	struct tm *tm;
	char *cp;

	jg = (struct jgpos *)sp;

	if (jg->navval != 0) {
		/*
		 * Solution not valid. Use caution and refuse
		 * to determine GPS week from this message.
		 */
		instance->gpos_gweek = 0;
		instance->gpos_sweek = 0;
		return ("Navigation solution not valid");
	}

	instance->gpos_gweek = jg->gweek;
	instance->gpos_sweek = DS2UI(jg->sweek);
	while(instance->gpos_sweek >= WEEKSECS) {
		instance->gpos_sweek -= WEEKSECS;
		++instance->gpos_gweek;
	}
	instance->gweek = 0;

	t = GPS_EPOCH + (instance->gpos_gweek * WEEKSECS) + instance->gpos_sweek;
	tm = gmtime(&t);
	cp = asctime(tm);

	javad_debug(instance->peer, __func__,
		"GPS %.24s (gweek/sweek %u/%u)",
		cp, instance->gpos_gweek, instance->gpos_sweek);
	return (NULL);
}
#endif

/*
 * javad_debug - print debug messages
 */
static void
javad_debug(struct peer *peer, const char *function, const char *fmt, ...)
{
	char	buffer[200];
	va_list	ap;

	va_start(ap, fmt);
	/*
	 * Print debug message to stdout
	 * In the future, we may want to get get more creative...
	 */
	mvsnprintf(buffer, sizeof(buffer), fmt, ap);
	record_clock_stats(&peer->srcadr, buffer);
#ifdef DEBUG
	if (debug) {
		printf("%s: %s\n", function, buffer);
		fflush(stdout);
	}
#endif

	va_end(ap);
}

static const char *
javad_send(struct instance *instance, const char *p)
{
	ssize_t size, cc;
	char buf[132];
	static char errmsg[132];

	size = msnprintf(buf, sizeof(buf), "%s\r", p);
	cc = write(instance->peer->procptr->io.fd, buf, size);
	if (cc < 0) {
		msnprintf(errmsg, sizeof(errmsg), "write: %m");
		return (errmsg);
	}
	if (cc != size) {
		snprintf(errmsg, sizeof(errmsg), "short write (%zd != %zd)",
		    cc, size);
		return (errmsg);
	}
	return (NULL);
}

#ifdef notdef
/* Request periodic message output */
static struct {
	struct jheader jheader;
	struct jrequest jrequest;
} reqmsg = {
	{ putshort(JAVAD_SYNC), 0,
	    putshort((sizeof(struct jrequest) / sizeof(u_short)) - 1),
	    0, JAVAD_FLAG_REQUEST | JAVAD_FLAG_NAK |
	    JAVAD_FLAG_CONN | JAVAD_FLAG_LOG, 0 },
	{ 0, 0, 0, 0 }
};
#endif

/* An interval of zero means to output on trigger */
static void
javad_reqmsg(struct instance *instance, u_int id,
    u_int interval)
{
#ifdef notdef
	struct jheader *hp;
	struct jrequest *rp;
	char *cp;

	hp = &reqmsg.jheader;
	hp->id = putshort(id);
	rp = &reqmsg.jrequest;
	rp->trigger = putshort(interval == 0);
	rp->interval = putshort(interval);
	if ((cp = javad_send(instance, hp)) != NULL)
		javad_debug(instance->peer, __func__, "%u: %s", id, cp);
#endif
}

#ifdef notdef
/* Cancel periodic message output */
static struct jheader canmsg = {
	putshort(JAVAD_SYNC), 0, 0, 0,
	JAVAD_FLAG_REQUEST | JAVAD_FLAG_NAK | JAVAD_FLAG_DISC,
	0
};
#endif

static void
javad_canmsg(struct instance *instance, u_int id)
{
#ifdef notdef
	struct jheader *hp;
	char *cp;

	hp = &canmsg;
	hp->id = putshort(id);
	if ((cp = javad_send(instance, hp)) != NULL)
		javad_debug(instance->peer, __func__, "%u: %s", id, cp);
#endif
}

#ifdef notdef
/* Request a single message output */
static struct jheader reqonemsg = {
	putshort(JAVAD_SYNC), 0, 0, 0,
	JAVAD_FLAG_REQUEST | JAVAD_FLAG_NAK | JAVAD_FLAG_QUERY,
	0
};
#endif

#ifdef notdef
/* Set the platform dynamics */
static struct {
	struct jheader jheader;
	struct jplat jplat;
} platmsg = {
	{ putshort(JAVAD_SYNC), putshort(JAVAD_I_PLAT),
	    putshort((sizeof(struct jplat) / sizeof(u_short)) - 1), 0,
	    JAVAD_FLAG_REQUEST | JAVAD_FLAG_NAK, 0 },
	{ 0, 0, 0 }
};
#endif

static void
javad_platform(struct instance *instance, u_int platform)
{
#ifdef notdef
	struct jheader *hp;
	struct jplat *pp;
	char *cp;

	hp = &platmsg.jheader;
	pp = &platmsg.jplat;
	pp->platform = putshort(platform);
	if ((cp = javad_send(instance, hp)) != NULL)
		javad_debug(instance->peer, __func__, "%u: %s", platform, cp);
#endif
}

/* Checksum "len" shorts */
static u_short
javad_cksum(u_short *sp, u_int len)
{
	u_short sum, x;

	sum = 0;
	while (len-- > 0) {
		x = *sp++;
		sum += getshort(x);
	}
	return (~sum + 1);
}

#ifdef notdef
/* Return the size of the next message (or zero if we don't have it all yet) */
static int
javad_recv(struct instance *instance)
{
	int n, len, size, cc;
	struct jheader *hp;
	u_char *bp;
	u_short *sp;

	/* Must have at least a header's worth */
	cc = sizeof(*hp);
	size = instance->ssize;
	if (size < cc)
		return (0);

	/* Search for the sync short if missing */
	sp = instance->sbuf;
	hp = (struct jheader *)sp;
	if (getshort(hp->sync) != JAVAD_SYNC) {
		/* Wasn't at the front, sync up */
		javad_debug(instance->peer, __func__, "syncing");
		bp = (u_char *)sp;
		n = size;
		while (n >= 2) {
			if (bp[0] != (JAVAD_SYNC & 0xff)) {
				/*
				javad_debug(instance->peer, __func__,
				    "{0x%x}", bp[0]);
				*/
				++bp;
				--n;
				continue;
			}
			if (bp[1] == ((JAVAD_SYNC >> 8) & 0xff))
				break;
			/*
			javad_debug(instance->peer, __func__,
			    "{0x%x 0x%x}", bp[0], bp[1]);
			*/
			bp += 2;
			n -= 2;
		}
		/*
		javad_debug(instance->peer, __func__, "\n");
		*/
		/* Shuffle data to front of input buffer */
		if (n > 0)
			memcpy(sp, bp, n);
		size = n;
		instance->ssize = size;
		if (size < cc || hp->sync != JAVAD_SYNC)
			return (0);
	}

	if (javad_cksum(sp, (cc / sizeof(u_short) - 1)) !=
	    getshort(hp->hsum)) {
	    javad_debug(instance->peer, __func__, "bad header checksum!");
		/* This is drastic but checksum errors should be rare */
		instance->ssize = 0;
		return (0);
	}

	/* Check for a payload */
	len = getshort(hp->len);
	if (len > 0) {
		n = (len + 1) * sizeof(u_short);
		/* Not enough data yet */
		if (size < cc + n)
			return (0);

		/* Check payload checksum */
		sp = (u_short *)(hp + 1);
		if (javad_cksum(sp, len) != getshort(sp[len])) {
			javad_debug(instance->peer,
			    __func__, "bad payload checksum!");
			/* This is drastic but checksum errors should be rare */
			instance->ssize = 0;
			return (0);
		}
		cc += n;
	}
	return (cc);
}
#endif

#else /* not (REFCLOCK && CLOCK_JAVAD && HAVE_PPSAPI) */
int refclock_javad_bs;
#endif /* not (REFCLOCK && CLOCK_JAVAD && HAVE_PPSAPI) */
