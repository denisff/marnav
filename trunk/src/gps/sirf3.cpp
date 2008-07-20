
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>

#include "sirf3.h"
//#include "udgps.h"

using namespace autoboat::hw;

// Define these consts here to make gcc4 happy
const int Sirf3::NMEA_MAX;
const int Sirf3::MAXTAGLEN;
const int Sirf3::MAXCHANNELS;

const int Sirf3::INPUT_BUF_SIZE;
const int Sirf3::OUTPUT_BUF_SIZE;
const unsigned int Sirf3::MAX_PACKET_LENGTH;
const int Sirf3::BAD_PACKET;
const int Sirf3::NO_PACKET;
const int Sirf3::NMEA_PACKET;

const double Sirf3::KNOTS_TO_MPS;	/* Knots to meters per second */

const double Sirf3::RMC_PERIOD;
const double Sirf3::GSA_PERIOD;
const double Sirf3::GGA_PERIOD;
const double Sirf3::GSV_PERIOD;


/**
 * Open the required device and establish communications with
 * the GPS.
 */

Sirf3::Sirf3(const char*input, int debug)
{
  debuglevel = debug;

  inputDevice = open( input, O_RDWR|O_NOCTTY);
  printf("inputDevice %d\n",inputDevice);
  if(inputDevice < 0)
  {
    fprintf(stderr, "failed to open gps device: %s\n", strerror(errno));
  }
  if(isatty(inputDevice)!=1)
  {
    fprintf(stderr, "Device not a tty device.\n");
  }
  report(2,"Device opened\n");

  tcgetattr(inputDevice, &ttyset);      //get current port settings

  ttyset.c_cflag &= ~(PARENB | PARODD | CRTSCTS);
  ttyset.c_cflag |= CREAD | CLOCAL;
  ttyset.c_iflag = ttyset.c_oflag = ttyset.c_lflag = (tcflag_t) 0;

  set_speed(4800);

#if (AUTO_BAUD > 0)
  printf("Speed set\n");
  printf("starting com test");
  if(commTest())
  {
    adjustBaudRate();
  }
  report(2,"Speed setup complete\n");
#endif

  RMCmode = 'V';
}

Sirf3::~Sirf3()
{
  if(close(inputDevice) != 0)
  {
    fprintf(stderr, "failed closing gps device: %s\n", strerror(errno));
  }
}

void Sirf3::setDebugLevel(int level)
{
  debuglevel = level;
}

int Sirf3::commTest()
{
  int maxTries = 10;

  packet_reset();

  int listenCount = 50;
  do
  {
    maxTries = 10;
    while((packetType != NMEA_PACKET) && (maxTries--))
    {
      report(5,"commTest: call packet_get\n");
      while(packet_get() == NO_PACKET)
        usleep(1000);
    }

    if(packetType == NMEA_PACKET)
    {
      report(5,"NMEA packet\n");
      if(nmea_parse() > 0)
      {
        return(0);
      }
    } else
       report(5,"non NMEA packet\n");
  } while( listenCount-- );

  return(-1);
}

void Sirf3::initialize()
{
}

void Sirf3::sendAndConfirm(char message[])
{
  do
  {
    nmea_send("%s",message);       //Keep sending message until acknowledged
  } while( waitForAck() );
}

int Sirf3::waitForAck()
{
  int maxTries = 100;
  int flag = 1;
  struct pollfd       pollStr[1];
  pollStr[0].fd = inputDevice;
  pollStr[0].events = POLLIN|POLLPRI;

  int retval;

  acknowledge = 0;
  while((maxTries-- > 0) )
    if((retval = poll(pollStr, 1, 100))>=0)
    {
      //printf("poll returned=%d revents=%x\n",retval, pollStr[0].revents);
      while(packet_get() == NO_PACKET)
        usleep(1000);
      if(packetType == NMEA_PACKET)
      {
        //report(5, "Good packet.\n");
        nmea_parse();
      }

      report(3,"Waiting for ACK\n");
      if(acknowledge)
      {
        flag = 0;
        break;
      }
    }

  return(flag);
}

int Sirf3::adjustBaudRate()
{
  int baudList[] = {115200, 4800, 9600, 19200, 38400, 57600 };
  int maxTries = 10;
  struct pollfd         wait[1];
  wait[0].fd = inputDevice;
  wait[0].events = POLLIN|POLLPRI;

  int xx = 0;
  report(5,"adjustBaudRate: search for correct Baud Rate\n");

  if((unsigned int)xx >= (sizeof(baudList)/sizeof(int)))
  {
    report(0,"adjustBaudRate: Failed to find BaudRate\n");
    return(-1);
  }

  set_speed(baudList[xx++]);

  int listenCount = 50;
  do
  {
    maxTries = 10;
    while((packetType != NMEA_PACKET) && (maxTries--))
    {
      while(packet_get() == NO_PACKET)
        usleep(1000);
    }

    if((packetType == NMEA_PACKET))
    {
      if(nmea_parse() > 3)
      {
        report(5,"adjustBaudRate: currentBaudRate=%d\n",baudList[xx++]);
        break;
      }
    }
  } while( listenCount-- );

  return(0);
}

int Sirf3::run(void)
{
  cycleComplete = 0;
  sentenceCount = 0;
  int status(0);

  //while(cycleComplete == 0)
  {
    packet_get();
    if(packetType == NMEA_PACKET)
    {
      status = nmea_parse();
    }
  }
  //report(3, "sentenceCount=%d\n", sentenceCount);
  return status;
}

/**
 * Flush out the serial buffer.
 */
void Sirf3::flush(void)
{
  tcflush(inputDevice,TCIOFLUSH);
}


int Sirf3::get_speed(void)
{
    speed_t code = cfgetospeed(&ttyset);
    switch (code) {
    case B0:     return(0);
    case B300:   return(300);
    case B1200:  return(1200);
    case B2400:  return(2400);
    case B4800:  return(4800);
    case B9600:  return(9600);
    case B19200: return(19200);
    case B38400: return(38400);
    case B57600: return(57600);
    default: return(115200);
    }
}

void Sirf3::set_speed( int newspeed)
{
    unsigned int	rate;
    if (newspeed < 300)
	rate = B0;
    else if (newspeed < 1200)
      rate =  B300;
    else if (newspeed < 2400)
      rate =  B1200;
    else if (newspeed < 4800)
      rate =  B2400;
    else if (newspeed < 9600)
      rate =  B4800;
    else if (newspeed < 19200)
      rate =  B9600;
    else if (newspeed < 38400)
      rate =  B19200;
    else if (newspeed < 57600)
      rate =  B38400;
    else if (newspeed < 115200)
      rate =  B57600;
    else
      rate =  B115200;

    if(cfsetispeed(&ttyset, B0) != 0)
      report(0, "Failed setting input speed\n");
    if(cfsetospeed(&ttyset, rate) != 0)
      report(0, "Failed setting output speed\n");

    ttyset.c_iflag &=~ (PARMRK | INPCK);
    ttyset.c_cflag &=~ (CSIZE | CSTOPB | PARENB | PARODD);
    //ttyset.c_cflag |= CS8 | PARENB | PARODD;
    ttyset.c_cflag |= CS8;

    if( tcsetattr(inputDevice, TCSANOW, &ttyset) != 0 )
      report(0, "Failed to configure serial device\n");

    flush();
    report(1, "set_speed speed=%d rate=%x\n", newspeed, rate);

    currentBaudRate = (unsigned int)newspeed;
    packet_reset();
}

void Sirf3::report(int errlevel, const char *fmt, ... )
{
    if (errlevel <= debuglevel) 
    {
	char buf[BUFSIZ], buf2[BUFSIZ], *sp;
	va_list ap;

	(void)strcpy(buf, "Sirf3: ");
	va_start(ap, fmt) ;
	(void)vsnprintf(buf + strlen(buf), sizeof(buf)-strlen(buf), fmt, ap);
	va_end(ap);

	buf2[0] = '\0';
	for (sp = buf; *sp != '\0'; sp++)
	    if (isprint(*sp) || (isspace(*sp) && (sp[1]=='\0' || sp[2]=='\0')))
		(void)snprintf(buf2+strlen(buf2), 2, "%c", *sp);
	    else
		(void)snprintf(buf2+strlen(buf2), 6, "\\x%02x", (unsigned)*sp);

        (void)fputs(buf2, stderr);
    }
}


enum {
  GROUND_STATE,	/* we don't know what packet type to expect */
  NMEA_DOLLAR,		/* we've seen first character of NMEA leader */
  NMEA_PUB_LEAD,	/* seen second character of NMEA G leader */
  NMEA_LEADER_END,	/* seen end char of NMEA leader, in body */
  NMEA_CR,	   	/* seen terminating \r of NMEA packet */
  NMEA_RECOGNIZED	/* saw trailing \n of NMEA packet */
};

char *Sirf3::gpsd_hexdump(void *binbuf, size_t binbuflen)
{
    static char hexbuf[MAX_PACKET_LENGTH*2+1];
    size_t i;
    size_t len = (size_t)((binbuflen > MAX_PACKET_LENGTH) ? MAX_PACKET_LENGTH : binbuflen);
    char *ibuf = (char *)binbuf;
    memset(hexbuf, 0, sizeof(hexbuf));

    for (i = 0; i < len; i++) {
	(void)snprintf(hexbuf + (2 * i), 3, "%02x", (unsigned int)(ibuf[i]&0xff));
    }
    return hexbuf;
}

/**
 * A sentence looks something like this:
 * $GPRMC,224759.85,A,3713.45089,N,12145.99453,W,000.04,161.59,280805,14.95,E,A*22
 * $GPRMC,224759.90,A,3713.45088,N,12145.99453,W,000.07,162.37,280805,14.95,E,A*2F 
 */
void Sirf3::nextstate(unsigned char c)
{
  switch(packetState)
  {
    case GROUND_STATE:
      if (c == '$') {
        packetState = NMEA_DOLLAR;
      }
      break;

    case NMEA_DOLLAR:
      if (c == 'G')
          packetState = NMEA_PUB_LEAD;
      else if (c == 'P')	/* vendor sentence */
          packetState = NMEA_LEADER_END;
      else
          packetState = GROUND_STATE;
      break;

    case NMEA_PUB_LEAD:
      if (c == 'P')
          packetState = NMEA_LEADER_END;
      else
          packetState = GROUND_STATE;
      break;

    case NMEA_LEADER_END:
      if (c == '\r')
          packetState = NMEA_CR;
      else if (c == '\n')
          /* not strictly correct, but helps for interpreting logfiles */
          packetState = NMEA_RECOGNIZED;
      else if (c == '$')
          /* faster recovery from missing sentence trailers */
          packetState = NMEA_DOLLAR;
      else if (!isprint(c))
          packetState = GROUND_STATE;
      break;

    case NMEA_CR:
      if (c == '\n')
          packetState = NMEA_RECOGNIZED;
      else
          packetState = GROUND_STATE;
      break;
        case NMEA_RECOGNIZED:
      if (c == '$')
          packetState = NMEA_DOLLAR;
      else
          packetState = GROUND_STATE;
      break;

  }
}

#define STATE_DEBUG

/* packet grab succeeded, move to output buffer */
void Sirf3::packet_accept(int packet_type)
{
    size_t packetlen = inbufptr - inbuffer;
    if (packetlen < sizeof(outbuffer)) 
    {
	memcpy((void *)outbuffer, (void *)inbuffer, packetlen);
	outbuflen = packetlen;
	outbuffer[packetlen] = '\0';
	packetType = packet_type;
#ifdef STATE_DEBUG
	report(6, "Packet type %d accepted %d = %s\n",
		packet_type, packetlen,
		gpsd_hexdump(outbuffer, outbuflen));
#endif /* STATE_DEBUG */
    } else {
	report(1, "Rejected too long packet type %d len %d\n",
		packet_type,packetlen);
    }
}

/* shift the input buffer to discard all data up to current input pointer */
void Sirf3::packet_discard()
{
    size_t discard = inbufptr - inbuffer;
    size_t remaining = inbuflen - discard;
    inbufptr = (unsigned char *)memmove(inbuffer,
                                         inbufptr,
                                         remaining);
    inbuflen = remaining;
#ifdef STATE_DEBUG
    report(6, "Packet discard of %d, chars remaining is %d = %s\n",
		discard, remaining,
		gpsd_hexdump(inbuffer, inbuflen));
#endif /* STATE_DEBUG */
}

/* shift the input buffer to discard one character and reread data */
void Sirf3::character_discard()
{
    memmove(inbuffer, inbuffer+1, (size_t)--inbuflen);
    inbufptr = inbuffer;
#ifdef STATE_DEBUG
    report(6, "Character discarded, buffer %d chars = %s\n",
		inbuflen,
		gpsd_hexdump(inbuffer, inbuflen));
#endif /* STATE_DEBUG */
}


/* entry points begin here */

/* get 0-origin big-endian words relative to start of packet buffer */
#define getword(i) (short)(session->inbuffer[2*(i)] | (session->inbuffer[2*(i)+1] << 8))


/* grab a packet; returns ether BAD_PACKET or the length */
ssize_t Sirf3::packet_parse(size_t newdata)
{
#ifdef STATE_DEBUG
    report(6, "Read %d chars to buffer offset %d (total %d): %s\n",
           newdata,
           inbuflen,
           inbuflen+newdata,
           gpsd_hexdump(inbufptr, newdata));
#endif /* STATE_DEBUG */

    outbuflen = 0;
    inbuflen += newdata;
#if 0
    inbuffer[inbuflen] = '\0';

  report(5, "Input buffer: %s\n", inbuffer);
#endif
    while (inbufptr < (inbuffer + inbuflen)) 
    {
      unsigned char c = *inbufptr++;
      static const char *state_table[] = {
        "GROUND_STATE",	/* we don't know what packet type to expect */
        "NMEA_DOLLAR",		/* we've seen first character of NMEA leader */
        "NMEA_PUB_LEAD",	/* seen second character of NMEA G leader */
        "NMEA_LEADER_END",	/* seen end char of NMEA leader, in body */
        "NMEA_CR",	   	/* seen terminating \r of NMEA packet */
        "NMEA_RECOGNIZED"	/* saw trailing \n of NMEA packet */
      };
      nextstate(c);
      report(7, "%08ld: character '%c' [%02x], new state: %s\n",
            char_counter, 
            (isprint(c)?c:'.'), 
            c, 
            state_table[packetState]);
      char_counter++;

      if (packetState == GROUND_STATE) 
      {
              character_discard();
      } 
      else if (packetState == NMEA_RECOGNIZED) 
      {
        bool checksum_ok = true;
        char csum[3];
        char *trailer = (char *)inbufptr-5;
        if (*trailer == '*') 
        {
          unsigned int n, crc = 0;
          for (n = 1; (char *)inbuffer + n < trailer; n++)
              crc ^= inbuffer[n];
          (void)snprintf(csum, sizeof(csum), "%02X", crc);
          checksum_ok = (toupper(csum[0])==toupper(trailer[1])
              && toupper(csum[1])==toupper(trailer[2]));
        }
        if (checksum_ok)
                packet_accept(NMEA_PACKET);

        packetState = GROUND_STATE;
        packet_discard();
        break;      // once we get a packet, get out
      }
    } /* while */

    return (ssize_t)newdata;
}
#undef getword

/* grab a packet; returns ether BAD_PACKET or the length */
ssize_t Sirf3::packet_get()
{
  ssize_t newdata;
  newdata = read(inputDevice, inbuffer+inbuflen, sizeof(inbuffer)-(inbuflen));
  if ((newdata < 0) && (errno != EAGAIN))
    return BAD_PACKET;
  else if ((newdata == 0) || ((newdata < 0) && (errno == EAGAIN)))
    return NO_PACKET;
  return packet_parse((size_t)newdata);
}

/* return the packet machine to the ground state */
void Sirf3::packet_reset()
{
    packetState = GROUND_STATE;
    inbuflen = 0;
    inbufptr = inbuffer;
    packetType = NO_PACKET;
}

/**
 * Functions for parsing NMEA sentences.
 */

/* process a pair of latitude/longitude fields starting at field index BEGIN */
void Sirf3::do_lat_lon(char *field[])
{
    double lat, lon, d, m;
    char str[20], *p;

    if (*(p = field[0]) != '\0') {
	strncpy(str, p, 20);
	(void)sscanf(p, "%lf", &lat);
	m = 100.0 * modf(lat / 100.0, &d);
	lat = d + m / 60.0;
	p = field[1];
	if (*p == 'S')
	    lat = -lat;
        latitude = lat;
    }
    if (*(p = field[2]) != '\0') {
	strncpy(str, p, 20);
	(void)sscanf(p, "%lf", &lon);
	m = 100.0 * modf(lon / 100.0, &d);
	lon = d + m / 60.0;

	p = field[3];
	if (*p == 'W')
	    lon = -lon;
        longitude = lon;
    }
  update_mask |= GPS_LATLONG_MASK;
}
/**************************************************************************
 *
 * NMEA sentence handling begins here
 *
 **************************************************************************/

#define DD(s)	((int)((s)[0]-'0')*10+(int)((s)[1]-'0'))

/* sentence supplied ddmmyy, but no century part */
void Sirf3::merge_ddmmyy(char *ddmmyy)
{
    date.tm_year = (2000 + DD(ddmmyy+4)) - 1900;
    date.tm_mon = DD(ddmmyy+2)-1;
    date.tm_mday = DD(ddmmyy);
}

/* update from a UTC time */
void Sirf3::merge_hhmmss(char *hhmmss)
{
    date.tm_hour = DD(hhmmss);
    date.tm_min = DD(hhmmss+2);
    date.tm_sec = DD(hhmmss+4);
    subseconds = atof(hhmmss+4) - date.tm_sec;
}

#undef DD

/* Global Positioning System Fix Data */
unsigned int Sirf3::processGPGGA(int count, char *field[])
{
    /*
        GGA,123519,4807.038,N,01131.324,E,1,08,0.9,545.4,M,46.9,M, , *42
           123519       Fix taken at 12:35:19 UTC
           4807.038,N   Latitude 48 deg 07.038' N
           01131.324,E  Longitude 11 deg 31.324' E
           1            Fix quality: 0 = invalid, 1 = GPS fix, 2 = DGPS fix,
	   		3=PPS (Precise Position Service),
			4=RTK (Real Time Kinematic) with fixed integers,
			5=Float RTK, 6=Estimated, 7=Manual, 8=Simulator
           08           Number of satellites being tracked
           0.9          Horizontal dilution of position
           545.4,M      Altitude, Metres above mean sea level
           46.9,M       Height of geoid (mean sea level) above WGS84
                        ellipsoid, in Meters
           (empty field) time in seconds since last DGPS update
           (empty field) DGPS station ID number (0000-1023)
    */

    fixQuality = atoi(field[6]);
    ggaCount = atoi(field[7]);
    //report(3, "GPGGA sets status %d\n", fixQuality);
    if (fixQuality > 0) 
    {
      altitude = atof(field[9]);
      statusFlags |= ALTITUDE_SET;
      update_mask |= GPS_ALTITUDE_MASK;

      separation = atof(field[11]);
    }
    sentenceCount++;
    return statusFlags;
}


unsigned int Sirf3::processGPGSA(int count, char *field[])
{
    /*
	eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
	eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35
	1    = Mode:
	       M=Manual, forced to operate in 2D or 3D
	       A=Automatic, 3D/2D
	2    = Mode: 1=Fix not available, 2=2D, 3=3D
	3-14 = PRNs of satellites used in position fix (null for unused fields)
	15   = PDOP
	16   = HDOP
	17   = VDOP
     */
    int i;

    /*
     * One chipset called the i.Trek M3 issues GPGSA lines that look like
     * this: "$GPGSA,A,1,,,,*32" when it has no fix.  This is broken
     * in at least two ways: it's got the wrong number of fields, and
     * it claims to be a valid sentence (A flag) when it isn't.
     * Alarmingly, it's possible this error may be generic to SiRF-IIIs.
     */
    if (count < 17)
	return ONLINE_SET;

    fixMode = atoi(field[2]);
    statusFlags = MODE_SET;
    //report(3, "GPGSA sets mode %d\n", session->gpsdata.newdata.mode);
    gsaCount = 0;
    memset(satUsed,0,sizeof(satUsed));
    for (i = 3; i < (count-3); i++) 
    {
      int prn = atoi(field[i]);
      satUsed[gsaCount++] = prn;
    }
    pdop = atof(field[i++]);
    hdop = atof(field[i++]);
    vdop = atof(field[i++]);
    //report(3, "GPGSA pdop=%lf hdop=%lf vdop=%lf\n", pdop, hdop, vdop);
    statusFlags |= HDOP_SET | VDOP_SET | PDOP_SET | USED_SET;

    sentenceCount++;
    return statusFlags;
}

/* Recommend Minimum Course Specific GPS/TRANSIT Data */
unsigned int Sirf3::processGPRMC(int count, char *field[] )
{
  /*
      RMC,225446.33,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E,A*68
    1.   225446.33    Time of fix 22:54:46 UTC
    2.   A            Status of Fix A = Autonomous, valid; 
                      D = Differential, valid; V = invalid
    3.   4916.45,N    Latitude 49 deg. 16.45 min North
    5.   12311.12,W   Longitude 123 deg. 11.12 min West
    7.   000.5        Speed over ground, Knots
    8.   054.7        Course Made Good, True north
    9    191194       Date of fix  19 November 1994
    10   020.3,E      Magnetic variation 20.3 deg East
    12.  A            FAA mode indicator (NMEA 2.3 and later)
                      A=autonomous, D=differential, E=Estimated,
                      N=not valid, S=Simulator, M=Manual input mode
         *68          mandatory nmea_checksum

   * SiRF chipsets don't return either Mode Indicator or magnetic variation.
   */
  if (strcmp(field[2], "V")==0) {
      RMCstatus = STATUS_NO_FIX;
      fixMode = MODE_NO_FIX;
      statusFlags |= ONLINE_SET|STATUS_SET|MODE_SET;
  } else if((strcmp(field[2], "A")==0)||(strcmp(field[2], "D")==0)) {
    RMCstatus = STATUS_FIX;
      merge_ddmmyy(field[9] );
      merge_hhmmss(field[1] );
      statusFlags |= TIME_SET;
      time = mkgmtime(&date)+(time_t)subseconds;

      do_lat_lon(&field[3]);
      statusFlags |= LATLON_SET;
      speed = atof(field[7]) * KNOTS_TO_MPS;
      track = atof(field[8]);
      statusFlags |= (TRACK_SET | SPEED_SET);
      update_mask |= GPS_SPEED_MASK;
      
      if(count >= 13)
      {
        double tmpMag = atof(field[10]);
        if(tmpMag > 0.0)
          statusFlags |= MAGVAR_SET;
        magVar = tmpMag;       /* magnetic variation */
        magDir = (strcmp(field[11], "E") == 0) ? 0:1; /* direction of variation */

        RMCmode = field[12][0];
      }
  }

  cycleComplete++;

  sentenceCount++;
  return statusFlags;
}


/* GPS Satellites in View */
unsigned int Sirf3::processGPGSV(int count, char *field[])
{
  /*
      GSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
         2            Number of sentences for full data
         1            sentence 1 of 2
         08           Total number of satellites in view
         01           Satellite PRN number
         40           Elevation, degrees
         083          Azimuth, degrees
         46           Signal-to-noise ratio in decibels
         <repeat for up to 4 satellites per sentence>
              There my be up to FOUR GSV sentences in a data packet
   */
  int fldnum;
  int startCount;

  /*
   * Start counting according to the message number.
   * You never know what the first packet you see is going to be.
   */
  startCount = (atoi(field[2]) - 1) * 4;
  gsvCount = atoi(field[3]);

  //report(3, "Counting %d satellites\n", session->gpsdata.satellites);
  for (fldnum = 4; fldnum < count; ) 
  {
      if (startCount >= MAXCHANNELS ) 
      {
          report(0, "internal error - too many satellites!\n");
          break;
      }

      PRN[startCount]       = atoi(field[fldnum++]);
      elevation[startCount] = atoi(field[fldnum++]);
      azimuth[startCount]   = atoi(field[fldnum++]);
      snr[startCount]        = atof(field[fldnum++]);

      startCount++;
  }

  sentenceCount++;
  return SATELLITE_SET;
}

/**************************************************************************
 *
 * Entry points begin here
 *
 **************************************************************************/

/* parse an NMEA sentence, unpack it into a session structure */
unsigned int Sirf3::nmea_parse()
{
    static struct {
	char *name;
        int funcNum;
    } nmea_phrase[] = {
	{"RMC", 1},
	{"GSV", 2},
	{"GSA", 3},
	{"GGA", 4},
    };
  char buf[NMEA_MAX+1];
  static char zeroStr[] = "0";

  int count;
  unsigned int retval = 0;
  unsigned int i;
  char *p, *field[NMEA_MAX], *s;

  /* make an editable copy of the sentence */
  strncpy(buf, (const char *)outbuffer, NMEA_MAX);

  /* discard the checksum part */
  for (p = buf; (*p != '*') && (*p >= ' '); ) 
    ++p;

  *p = 0;
  /* split sentence copy on commas, filling the field array */

  for (count = 0, p = (char *)buf; (p != 0) && (*p != 0); p = strchr(p, ',')) 
  {
    *p = 0;
    ++p;
    if((*p) != ',')
    {
      field[count] = p;
    }
    else
      field[count] = zeroStr;

    ++count;
  }

  /* dispatch on field zero, the sentence tag */
  for (i = 0; i < (unsigned)(sizeof(nmea_phrase)/sizeof(nmea_phrase[0])); ++i) 
  {
    s = field[0];
    if (strlen(nmea_phrase[i].name) == 3)
	    s += 2;	/* skip talker ID */
        if (strcmp(nmea_phrase[i].name, s) == 0) {
          switch(nmea_phrase[i].funcNum)
          {
            case 1:
              processGPRMC(count, field);
              retval = 1;
              break;
            case 2:
              processGPGSV(count, field);
              retval = 2;
              break;
            case 3:
              processGPGSA(count, field);
              retval = 3;
              break;
            case 4:
              processGPGGA(count, field);
              retval = 4;
              break;
            default:
              retval = 0;
              break;
          }

          strncpy(tag, nmea_phrase[i].name, MAXTAGLEN);
          sentenceLength = strlen((const char *)outbuffer);
          report(5,"Got Packet: type=%s\n", tag);
          break;
	}
    }
    packetType = NO_PACKET;
    return retval;
}

/* add NMEA checksum to a possibly  *-terminated sentence */
void Sirf3::nmea_add_checksum(char *sentence)
{
    unsigned char sum = '\0';
    char c, *p = sentence;

    if (*p == '$') {
	p++;
    } else {
        report(1, "Bad NMEA sentence: '%s'\n", sentence);
    }
    while ( ((c = *p) != '*') && (c != '\0')) {
	sum ^= c;
	p++;
    }
    *p++ = '*';
    (void)snprintf(p, 5, "%02X\r\n", (unsigned)sum);
}

/* ship a command to the GPS, adding * and correct checksum */
int Sirf3::nmea_send(const char *fmt, ... )
{
    int status;
    char buf[BUFSIZ];
    va_list ap;

    va_start(ap, fmt) ;
    (void)vsnprintf(buf, sizeof(buf)-5, fmt, ap);
    va_end(ap);
    if (fmt[0] == '$') {
	strcat(buf, "*");
	nmea_add_checksum(buf);
    } else
	strcat(buf, "\r\n");
    status = (int)write(inputDevice, buf, strlen(buf));
    if (status == (int)strlen(buf)) {
	report(3, "=> GPS: %s\n", buf);
	return status;
    } else {
	report(3, "=> GPS: %s FAILED\n", buf);
	return -1;
    }
}


/* struct tm to seconds since Unix epoch */
time_t Sirf3::mkgmtime(register struct tm *t)
{
    const int MONTHSPERYEAR = 12;
    register int year;
    register time_t result;
    static const int cumdays[MONTHSPERYEAR] =
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

    /*@ +matchanyintegral @*/
    year = 1900 + t->tm_year + t->tm_mon / MONTHSPERYEAR;
    result = (year - 1970) * 365 + cumdays[t->tm_mon % MONTHSPERYEAR];
    result += (year - 1968) / 4;
    result -= (year - 1900) / 100;
    result += (year - 1600) / 400;
    result += t->tm_mday - 1;
    result *= 24;
    result += t->tm_hour;
    result *= 60;
    result += t->tm_min;
    result *= 60;
    result += t->tm_sec;
    /*@ -matchanyintegral @*/
    return (result);
}

