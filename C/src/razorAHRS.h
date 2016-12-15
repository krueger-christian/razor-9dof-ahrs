/*  
 *  @file   razorAHRS.c   
 *  @author Christian Krüger
 *  @date   26.10.2016
 *  @organisation Quality & Usablity Lab, T-Labs, TU Berlin
 *
 *  parses serial streamed data of the 9 Degree of 
 *  Measurement Attitude and Heading Reference System 
 *  of Sparkfun's "9DOF Razor IMU" and "9DOF Sensor Stick"
 *
 *  based on a parser written in C++ by Peter Bartz:
 *  https://github.com/ptrbrtz/razor-9dof-ahrs
 *
 *  for further informations check the README file 
 */


#ifndef RAZORAHRS_H
#define RAZORAHRS_H

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>


/* disable/enable the output of status reports on the console */
#define message_on 1

/* disable/enable extra status reports, e.g. buffer content */
#define debug 0

/* use only extended magnetometer calibration mode */
#define EXT_MAGN_CAL 1 

/* color codes for colored text on stdout */
#define COL_NORMAL 	"\x1B[0m"
#define COL_RED		"\x1B[31m"
#define COL_GREEN	"\x1B[32m"
#define COL_YELLOW	"\x1B[33m"
#define COL_BLUE	"\x1B[34m"
#define COL_MAGENTA	"\x1B[35m"
#define COL_CYAN	"\x1B[36m"
#define COL_WHITE	"\x1B[37m"


/*----------------------------------------------------------------------------------------------------*/

	// streaming mode could be single or continous
	enum streamingMode
	{
		STREAMINGMODE_ONREQUEST, 
		STREAMINGMODE_CONTINUOUS,
	};

/*----------------------------------------------------------------------------------------------------*/

	/*  text: char array with undefined length, starts with "YPR" and terminates with "\n"
	 *  binary: 12 Byte format, equals 3 float values
	 *  32bit: 4 Byte format, equal to the long format but isn't readable as a long, containing 
	 *         all three sensor angles as kind of a 10 Bit "integer"
     */
	enum streamingFormat 
	{
		STREAMINGFORMAT_ASCII, 
		STREAMINGFORMAT_BINARY_FLOAT
	};

/*----------------------------------------------------------------------------------------------------*/

	enum calibrationStep
	{
		X_MAX,   // x-axis maximum
		X_MIN,   // x-axis minimum
		Y_MAX,   // y-axis maximum
		Y_MIN,   // y-axis minimum
		Z_MAX,   // z-axis maximum
		Z_MIN    // z-axis minimum
	};

/*----------------------------------------------------------------------------------------------------*/

	enum sensorType
	{
		ACC,   // accelerometer
		MAG,   // magnetometer
		GYR    // gyrometer
	};

/*----------------------------------------------------------------------------------------------------*/

	/* The struggle is, to transform the binary data of the serial stream
	 * into useful values. It's just a matter of interpretting ones
	 * and zeros. So we can use the same memory location and don't care if
	 * we put four characters inside, one float value or the razor specific 32 bit 
	 * data format. All cases require four bytes. We do this using a union structure... */ 
	union razorBuffer
	{
		float f;
		char ch[4];
		long l;
	};

/*----------------------------------------------------------------------------------------------------*/

	struct razorSetup
	{
		struct termios old_tio;
		enum streamingFormat streaming_Format;
		enum streamingMode streaming_Mode;

		bool messageOn;
		bool tio_config_changed;
		bool synchronized;
		bool *tracker_should_exit;

		char *port;
    	int tty_fd;
		int waitingTime;
		speed_t baudRate;
	};

/*----------------------------------------------------------------------------------------------------*/

	struct thread_parameter
	{
		struct razorSetup*  setup;
		struct razorData*   data;

		pthread_mutex_t setup_protect;
		pthread_mutex_t data_protect;

		pthread_cond_t data_updated;
		pthread_cond_t update;

		int thread_id;
		pthread_t thread;

		bool dataUpdated;
		bool razor_is_running;
		bool printer_is_running;
	};

/*----------------------------------------------------------------------------------------------------*/

	/* data type to store the yaw, pitch and roll
	   value of the RazorAHRS tracker */
	struct razorData
	{
		/* Flag that is managed by valueCheck() function
		 * true: the current values don't match the valid range
		 * false: the current values are inside the valid range */
		bool data_fail;

		/* Flag to signal if someone requests an update of the data */
		bool dataRequest;

		/* array that stores the sensor data
		 *
		 * values[0] = Yaw
		 * values[1] = Pitch
	 	 * values[2] = Roll  */
		float values[3];

		/* union structure to store blocks of 
		 *  4 Byte received by the tracker (equal 
		 *  to the size of a single float or long value.
		 *  for further information look at 
		 *  razorTools.h */
		union razorBuffer buffer;
	};

/*----------------------------------------------------------------------------------------------------*/

long elapsed_ms(struct timeval start, struct timeval end);
void razorSleep(int milliseconds);
void tio_Config(int tty_fd, speed_t baudRate);
void resetConfig(struct razorSetup *setup);

bool  synch           ( struct thread_parameter *parameter);
bool  valueCheck      ( struct thread_parameter *parameter );

bool  readContinuously( struct thread_parameter *parameter );
bool  readOnRequest   ( struct thread_parameter *parameter );
void* readingRazor    ( struct thread_parameter *parameter );

struct thread_parameter* razorAHRS ( speed_t baudRate, char* port, int mode, int format );
int   razorAHRS_start   ( struct thread_parameter *parameter );
int   razorAHRS_quit    ( struct thread_parameter *parameter );
void  razorAHRS_stop    ( struct thread_parameter *parameter );
int   razorAHRS_request ( struct thread_parameter *parameter );

void* razorPrinter             ( void* args);
void  razorPrinter_start       ( struct thread_parameter *parameter, pthread_t *printer);
int   razorPrinter_stop        ( struct thread_parameter *parameter);

#endif // RAZORAHRS_H
