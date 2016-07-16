/* Comm Interface test Program */

/* -b xxxx set baud rate
   [ 7 | 8 ][n | e | o ][1 | 2], 
*/
#include <stdio.h>      // for FILE
#include <unistd.h>
#include <stdlib.h>     // for timeval
#include <string.h>     // for strlen etc
#include <time.h>       // for ctime
#include <sys/types.h>  // for fd_set
#include <netdb.h>      // for sockaddr_in 
#include <fcntl.h>      // for O_RDWR
#include <termios.h>    // for termios
#include <getopt.h>             // for getopt
#ifdef linux
#include <errno.h>              // for Linux
#endif

/* Version 0.0 0709 Created by copying from Fronius-test */

#define VERSION "0.0"

#ifdef linux
#define SERIALNAME "/dev/ttyAM0"        /* although it can be supplied on command line */
#else
#define SERIALNAME "/dev/cu.usbserial-FTCTD7JM"        /* although it can be supplied on command line */
#endif
// Serial retry params
#define SERIALNUMRETRIES 10
#define SERIALRETRYDELAY 1000000 /*microseconds */
#define WAITTIME 3      /*seconds*/
// Set to if(0) to disable debugging
#define DEBUG if(debug)
#define DEBUG2 if(debug > 1)
#define DEBUGFP stderr   /* set to stderr or logfp as required */

// This allows use of stdio instead of the serial device, and you can type in the hex value
// #define DEBUGCOMMS

#ifndef linux
extern
#endif
int errno; 

// Procedures used
int openSerial(const char * name, int baud, int c_cflag);  // return fd
void closeSerial(int fd);  // restore terminal settings
int sendSerial(int fd, unsigned char data);     // Send a byte
void usage(void);                                               // Standard usage message
void processComm(int commfd);


// Globals
int debug = 1;

//#define BUFSIZE 13      /* A packet is up to 12 bytes */
//unsigned char serialbuf[BUFSIZE];       // data accumulates in this global
//int serbufindex = 0;

//#define LOBYTE(x) (x & 0xFF)
//#define HIBYTE(x) ((x & 0xFF00) >> 8)

/********/
/* MAIN */
/********/
int main(int argc, char *argv[])
// arg1: serial device file
// arg2: optional timeout in seconds, default 60
// arg3: optional 'nolog' to carry on when filesystem full
{
    int commfd = 0;
	char ch;
	
	int c_cflag = 0;
	int done = 0;
	int option;                             // command line processing
	time_t commandSent;
	int baud = 300;
	
    char * serialName = SERIALNAME;
	
	fd_set readfd; 
	int numfds;
	struct timeval timeout;
	
	// Command line arguments
	
	opterr = 0;
	while ((option = getopt(argc, argv, "1278b:dVs:?one")) != -1) {
		DEBUG fprintf(stderr, "Option %c ", option);
		switch (option) {
			case '7': c_cflag |= CS7; break;
			case '8': c_cflag |= CS8; break;
			case 'n': c_cflag &= ~PARENB; break;		// NO parity
			case 'e': c_cflag |= PARENB; break;			// EVEN parity
			case 'o': c_cflag |= PARENB | PARODD; break;	// ODD parity
			case '1': c_cflag &= ~CSTOPB; break;		// 1 stop bit
			case '2': c_cflag |= CSTOPB; break;			// 2 stop bits
			case 'b': baud = atoi(optarg); break;
			case '?': usage(); exit(1);
			case 's': serialName = optarg; break;
			case 'd': debug = 1; break;
			case 'V': printf("Version: %s\n", VERSION); exit(0);
		}
	}
	
	// Open serial port
#ifdef DEBUGCOMMS
	commfd = 0;
#else
	DEBUG fprintf(stderr, "Starting .. \n");
	
	if ((commfd = openSerial(serialName, baud, c_cflag)) < 0) {
		fprintf(stderr, "ERROR Meter Failed to open %s at %d: %s", serialName, baud, strerror(errno));
		exit(1);
	}
	
#endif
	DEBUG fprintf(stderr, "Serial open .. \n");
	sleep(1);
	
	numfds = commfd + 1;              // nfds parameter to select. One more than highest descriptor
									  // Temp trying +10 instead of +1
									  // Main Loop
	FD_ZERO(&readfd); 
	commandSent = time(NULL);
	
	while(!done) {
		//      sendInit(commfd);
		timeout.tv_sec = WAITTIME;		// wait up to three seconds for first character
		timeout.tv_usec = 0;
		
		FD_SET(commfd, &readfd);
		FD_SET(STDIN_FILENO, &readfd);
		select(numfds, &readfd, NULL, NULL, &timeout);
		// Want to loop here consuming anything from the Fronius
		if (FD_ISSET(STDIN_FILENO, &readfd)) {
			read(STDIN_FILENO, &ch, 1);
			sendSerial(commfd, ch);
			DEBUG fprintf(stderr, "[%02x]<< ", ch);
			if (ch == 4)
				done = 1;
		}
		if (FD_ISSET(commfd, &readfd)) {
			read(commfd, &ch, 1);
			// putc(ch, stderr);
			DEBUG fprintf(stderr, "[%02x]>> ", ch);
			if (ch == 4)
				done = 1;
		}
	}
	
	
	closeSerial(commfd);
	fprintf(stderr, "\n");
	
	return 0;
}

/*********/
/* USAGE */
/*********/
void usage(void) {
        printf("Usage: comm [-b 300|600|1200|2400|4800] [-s serialdev]\n");
		printf("  -7|8 : bits -n|o|e parity none, odd, even  -1|2 stop bits\n");
        return;
}

unsigned char evenparity(unsigned char data) {
	int i, p;
	p = 0;
	for (i = 0; i < 8; i++) if (data & (1 << i)) p++;
	if (p % 2 == 1) data += 128;
	return data;
}

/**************/
/* SENDSERIAL */
/**************/
int sendSerial(int fd, unsigned char data) {
	// Send a single byte.  Return 1 for a logged failure
	int retries = SERIALNUMRETRIES;
	int written;
#ifdef DEBUGCOMMS
	fprintf(stderr, "Comm 0x%02x(%d) ", data, data);
	return 0;
#endif

	//	data = evenparity(data);
	DEBUG fprintf(stderr, "[%02x]>> ", data);
	while ((written = write(fd, &data, 1)) < 1) {
        fprintf(DEBUGFP, "Serial wrote %d bytes errno = %d", written, errno);
        perror("");
		if (--retries == 0) {
			fprintf(stderr, "WARN Fronius timed out writing to serial port");
			return 1;
		}
		DEBUG fprintf(DEBUGFP, "Pausing %d ... ", SERIALRETRYDELAY);
		usleep(SERIALRETRYDELAY);
	}
	return 0;       // ok
}

/**************/
/* OPENSERIAL */
/**************/
int openSerial(const char * name, int baud, int c_cflag) {
	/* open serial device; return file descriptor or -1 for error (see errno) */
	int fd, res, speed;
	struct termios newSettings;
	
	if ((fd = open(name, O_RDWR | O_NOCTTY)) < 0) return fd;        // an error code
	
	bzero(&newSettings, sizeof(newSettings));
	// Control Modes
	newSettings.c_cflag = /* CRTSCTS  |*/ c_cflag | CLOCAL | CREAD; 
	// input modes
	newSettings.c_iflag = /* IGNPAR */PARMRK | INPCK ;   //input modes
	newSettings.c_oflag = 0;                // output modes
	newSettings.c_lflag = 0;                // local flag
	newSettings.c_cc[VTIME] = 0; // intercharacter timer */
    newSettings.c_cc[VMIN] = 0;     // non-blocking read */
	if (baud)	{
		switch(baud) {
		case 300: speed = B300; break;
		case 600: speed = B600; break;
		case 1200: speed = B1200; break;
		case 2400: speed = B2400; break;
		case 4800: speed = B4800; break;
		case 9600: speed = B9600; break;
		default: fprintf(stderr, "Error - Baud must be 2400 or 19200 - %d supplied\n", baud);
		}
		cfsetspeed(&newSettings, speed);
	}
	DEBUG fprintf(stderr," C_CFLAG = %04x C_IFLAG=%04x C_OFLAG=%04x\n", newSettings.c_cflag,newSettings.c_iflag,newSettings.c_oflag);
	tcflush(fd, TCIFLUSH);          // discard pending data
											//      cfsetospeed(&newSettings, baud);
	if((res = tcsetattr(fd, TCSANOW, &newSettings)) < 0) {
		close(fd);      // if there's an error setting values, return the error code
		return res;
	}
	return fd;
}

/***************/
/* CLOSESERIAL */
/***************/
void closeSerial(int fd) {
	close(fd);
}


/***************/
/* PROCESSCOMM */
/***************/
void processComm(int commfd) {
	// Deal with just one byte.
	unsigned char thischar;

#ifdef DEBUGCOMMS
	{  int val;
        gets(buf);
        val = strtol(buf, NULL, 16);
        fprintf(stderr, "\nGot %U ", val);
        thischar = val;
	}
#else
	DEBUG2 fprintf(stderr, "Processcomm: ");
	      read(commfd, &thischar, 1);       // must be able to read at least one byte otherwise the 
															// fd would not be readable.
	
#endif
	 fprintf(stderr, "%02x ", thischar);
};

	