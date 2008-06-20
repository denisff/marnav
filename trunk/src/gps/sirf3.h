#pragma once
/**
 * This is a simple driver for the Sirf3 based GPS.
 *
 * - Use Baud rate 4800bps, 8N1.
 */
#include <string.h>
#include <termios.h>
#include <time.h>

namespace autoboat
{
namespace hw
{
class Sirf3
{
  public:
    static const int NMEA_MAX = 120;
    static const int MAXTAGLEN = 8;
    static const int MAXCHANNELS = 16;

    static const int INPUT_BUF_SIZE = 128;
    static const int OUTPUT_BUF_SIZE = 128;
    static const unsigned int MAX_PACKET_LENGTH = 120;
    static const int BAD_PACKET = -1;
    static const int NO_PACKET = 0;
    static const int NMEA_PACKET = 1;

    static const double KNOTS_TO_MPS =	0.51444444;	/* Knots to meters per second */

    static const double RMC_PERIOD = 0.10;
    static const double GSA_PERIOD = 0.10;
    static const double GGA_PERIOD = 0.10;
    static const double GSV_PERIOD = 1.0;

    Sirf3(const char*input=NULL, int debug=0);
    ~Sirf3();

    int run();
    void setDebugLevel(int);
    int  commTest();
    void initialize();

  private:
    void flush(void);                //Flushes the serial buffer
    int get_speed(void);
    void set_speed( int speed);
    int adjustBaudRate();
    void sendAndConfirm(char message[]);
    int waitForAck();
    void report(int errlevel, const char *fmt, ... );
    char *gpsd_hexdump(void *binbuf, size_t binbuflen);
    void nextstate(unsigned char c);
    void packet_accept(int packet_type);
    void packet_discard();
    void character_discard();
    ssize_t packet_parse(size_t newdata);
    ssize_t packet_get();
    void packet_reset();
    void do_lat_lon(char *field[]);
    int nmea_send(const char *fmt, ... );
    void nmea_add_checksum(char *sentence);
    time_t mkgmtime(register struct tm *t);
    void merge_ddmmyy(char *ddmmyy);
    void merge_hhmmss(char *hhmmss);
    unsigned int nmea_parse();
    unsigned int processGPGSV(int count, char *field[]);
    unsigned int processGPRMC(int count, char *field[] );
    unsigned int processGPGSA(int count, char *field[]);
    unsigned int processGPGGA(int count, char *field[]);

  private:
    int                 inputDevice;
    struct termios      ttyset;
    int                 currentBaudRate;
    int                 currentBaudRateIndex;
    int                 currentPort;    //Port on GPS A=0, B=1, C=2
    char                *idString;
    int debuglevel;
    int sentenceCount;

    //Parsing stuff
    int packetState,    // Current statemachine location
        packetType;     // Type of packet just identified
    unsigned char inbuffer[INPUT_BUF_SIZE+1];
    unsigned char outbuffer[OUTPUT_BUF_SIZE+1];
    size_t inbuflen,outbuflen;
    unsigned char *inbufptr;
    unsigned long char_counter;
    char tag[MAXTAGLEN];
    int sentenceLength;
    int cycleComplete;
    int acknowledge;   // set this true when we receive a valid acnowledge

  public:       //Expose this for easy access to results
    unsigned int statusFlags;
    //These are flags for status above
    static const unsigned int ONLINE_SET	= 0x00000001u;
    static const unsigned int TIME_SET	        = 0x00000002u;
    static const unsigned int TIMERR_SET	= 0x00000004u;
    static const unsigned int LATLON_SET	= 0x00000008u;
    static const unsigned int ALTITUDE_SET	= 0x00000010u;
    static const unsigned int SPEED_SET	        = 0x00000020u;
    static const unsigned int TRACK_SET	        = 0x00000040u;
    static const unsigned int CLIMB_SET	        = 0x00000080u;
    static const unsigned int STATUS_SET	= 0x00000100u;
    static const unsigned int MODE_SET	        = 0x00000200u;
    static const unsigned int HDOP_SET  	= 0x00000400u;
    static const unsigned int VDOP_SET  	= 0x00000800u;
    static const unsigned int PDOP_SET  	= 0x00001000u;
    static const unsigned int TDOP_SET	        = 0x00002000u;
    static const unsigned int GDOP_SET	        = 0x00004000u;
    static const unsigned int DOP_SET		= (HDOP_SET|VDOP_SET|PDOP_SET|TDOP_SET|GDOP_SET);
    static const unsigned int HERR_SET	        = 0x00008000u;
    static const unsigned int VERR_SET	        = 0x00010000u;
    static const unsigned int PERR_SET	        = 0x00020000u;
    static const unsigned int SATELLITE_SET	= 0x00040000u;
    static const unsigned int PSEUDORANGE_SET	= 0x00080000u;
    static const unsigned int USED_SET	        = 0x00100000u;
    static const unsigned int SPEEDERR_SET	= 0x00200000u;
    static const unsigned int TRACKERR_SET	= 0x00400000u;
    static const unsigned int CLIMBERR_SET	= 0x00800000u;
    static const unsigned int DEVICE_SET	= 0x01000000u;
    static const unsigned int DEVICELIST_SET	= 0x02000000u;
    static const unsigned int DEVICEID_SET	= 0x04000000u;
    static const unsigned int ERROR_SET	        = 0x08000000u;
    static const unsigned int CYCLE_START_SET	= 0x10000000u;
    static const unsigned int RTCM_SET	        = 0x20000000u;
    static const unsigned int MAGVAR_SET	= 0x40000000u;
    static const unsigned int FIX_SET		= (TIME_SET|MODE_SET|TIMERR_SET|LATLON_SET|HERR_SET|ALTITUDE_SET|VERR_SET|TRACK_SET|TRACKERR_SET|SPEED_SET|SPEEDERR_SET|CLIMB_SET|CLIMBERR_SET);

    unsigned int fixMode;
    static const unsigned int MODE_NOT_SEEN	= 0;	/* mode update not seen yet */
    static const unsigned int MODE_NO_FIX	= 1;	/* none */
    static const unsigned int MODE_2D  	        = 2;	/* good for latitude/longitude */
    static const unsigned int MODE_3D  	        = 3;	/* good for altitude/climb too */

    unsigned int RMCstatus;
    //These are for RMCstatus above
    static const int STATUS_NO_FIX = 0;
    static const int STATUS_FIX = 1;

    int fixQuality;     //Taken from GGA sentence
    struct tm   date;
    time_t time;        //formatted as Unix time
    double subseconds;
    double latitude;
    double longitude;
    double altitude;
    double separation;
    double speed;
    double track;
    double magVar;
    int    magDir;
    char   RMCmode;
    double pdop,hdop,vdop;

    int gsvCount;       // Sattelite count according to GSV
    int gsaCount;
    int ggaCount;       // Number of sats used according to GGA
    int satUsed[MAXCHANNELS];
    int PRN[MAXCHANNELS];
    int elevation[MAXCHANNELS];
    int azimuth[MAXCHANNELS];
    double snr[MAXCHANNELS];

    int update_mask;
};
}
}
