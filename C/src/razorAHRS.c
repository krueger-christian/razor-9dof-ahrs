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


#ifndef RAZORAHRS_C
#define RAZORAHRS_C

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>

#include "razorAHRS.h"


/* time limit (milliseconds) to wait during synchronization*/
const int connect_timeout_ms = 5000;

const char *token_reference = "#SYNCH00\r\n"; 
const int token_length = 10;

/*----------------------------------------------------------------------------------------------------*/

/* measuring elapsed time:
 * During synchronization we need to check the time. Ones to know,
 * when we have to send a new request to the tracker/board, second 
 * to stop the attempt after a certain amount of time, when we could 
 * be shure not having success anymore.   
 */
long elapsed_ms(struct timeval start, struct timeval end)
{
	return (long) ((end.tv_sec - start.tv_sec) * 1000 + (end.tv_usec - start.tv_usec) / 1000);
}

/*----------------------------------------------------------------------------------------------------*/

void razorSleep(int milliseconds)
{
	struct timespec waitTime;
	waitTime.tv_sec = milliseconds / 1000;
	waitTime.tv_nsec = (milliseconds % 1000) * 1000000;
	nanosleep(&waitTime, NULL);
}

/*----------------------------------------------------------------------------------------------------*/

void tio_Config(int tty_fd, speed_t baudRate) 
{
    struct termios tio;

    memset(&tio, 0, sizeof (tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 10;
    cfsetospeed(&tio, baudRate); // 57600 baud
    cfsetispeed(&tio, baudRate); // 57600 baud
    tcsetattr(tty_fd, TCSANOW, &tio);
}

/*----------------------------------------------------------------------------------------------------*/

void resetConfig(struct razorSetup *setup) 
{
    if (setup->tio_config_changed) tcsetattr(setup->tty_fd, TCSANOW, &setup->old_tio);
    close(setup->tty_fd);
}

/*----------------------------------------------------------------------------------------------------*/

// function to synchronize incoming bytes with the tracker
bool synch( struct thread_parameter *parameter)
{
    if (parameter->setup->messageOn) printf("  !\n    SYNCHRONIZING: ");

    char input = 'D'; // Buffer to store one byte

    char* token = calloc(token_length, sizeof (char));

    tcflush(parameter->setup->tty_fd, TCIFLUSH);

    for(int i = 0; (i <= 10) && (write(parameter->setup->tty_fd, "#s00", 4) != 4); i++)
        razorSleep(20);

    struct timeval t0, t1, t2;
    gettimeofday(&t0, NULL);
    t1 = t0;

    size_t token_pos = 0;

	/* SYNCHRONIZATION
     * Looking for correct token. */
    while (1)
	{
        // input available?
        if (read(parameter->setup->tty_fd, &input, 1) > 0) 
		{
			// Does it match with first character of the token?            
			if (input == '#')
			{
                token_pos++;

                token[0] = input;

                // ... first byte matchs, so get the next bytes
                while (token_pos < token_length) 
				{
                    if (read(parameter->setup->tty_fd, &input, 1) > 0) 
					{
                        token[token_pos] = input;
                        token_pos++;
                    }
                }

                /* if received token is equal to the reference token
                 * the variable "synchronized" is set to true */
                parameter->setup->synchronized = (strncmp(token, token_reference,10) == 0) ? true : false;

                if (parameter->setup->synchronized == true)
				{
                    if (parameter->setup->messageOn) printf("%s__okay.\n\n\r", COL_GREEN);
                    free(token);
                    return true;
                }
            }
        }

        gettimeofday(&t2, NULL);
        if (elapsed_ms(t1, t2) > 200)
		{
            // 200ms elapsed since last request and no answer -> request synch again
            // (this happens when DTR is connected and Razor resets on connect)

			tcflush(parameter->setup->tty_fd, TCIFLUSH);

            for(int i = 0; (i <= 10) && (write(parameter->setup->tty_fd, "#s00", 4) != 4); i++)
				razorSleep(20);

            t1 = t2;
        }
		//timeout?
        if (elapsed_ms(t0, t2) > connect_timeout_ms)
		{
            parameter->setup->synchronized = false;
            if (parameter->setup->messageOn) printf("___failed. (time out)\n\n\r"); // TIME OUT!		
            free(token);
            return false;
        }
        token_pos = 0;
    }
}

/*----------------------------------------------------------------------------------------------------*/

bool valueCheck( struct thread_parameter *parameter)
{
	int yaw   = parameter->data->values[0];
	int pitch = parameter->data->values[1];
	int roll  = parameter->data->values[2];

	if(yaw < 0)   yaw   *= -1;
	if(pitch < 0) pitch *= -1;
	if(roll < 0)  roll  *= -1;

    if ((yaw > 360) || (pitch > 360) || (roll > 360))
	{
		parameter->data->data_fail = true;
    }
	else parameter->data->data_fail = false;

    return !(parameter->data->data_fail);
}

/*----------------------------------------------------------------------------------------------------*/

bool readContinuously( struct thread_parameter *parameter)
{
    char singleByte = 'D';
    int result = 0;
    int bufferInput = 0;
    size_t values_pos = 0;

	// ensure that currently only this function changes razor setup
	pthread_mutex_lock(&parameter->setup_protect);

	// enable output continuous stream
    write(parameter->setup->tty_fd, "#o1", 3);
    razorSleep(20);

	// define output mode
	write(parameter->setup->tty_fd, "#ob", 3);
	razorSleep(20);

    tcflush(parameter->setup->tty_fd, TCIFLUSH);

	if(synch(parameter) == false)
	{
		pthread_mutex_unlock(&parameter->setup_protect);		
		return false;
	}

	while(parameter->razor_is_running)
	{
        result = read(parameter->setup->tty_fd, &singleByte, 1);

		#if debug        
			printf("Inside buffer: %d \t read bytes: %d\n\r", bufferInput, result);
		#endif
 
        if (result == 1)
		{
			// ensure that currently only this function changes razor data
			pthread_mutex_lock(&parameter->data_protect);
            parameter->data->buffer.ch[bufferInput] = singleByte;
            bufferInput++;

            if (bufferInput == 4)
			{
		        parameter->data->values[values_pos] = parameter->data->buffer.f;
		        values_pos++;
		        bufferInput = 0;
			}
			pthread_mutex_unlock(&parameter->data_protect);
        }
		else if((result > 1) && parameter->setup->messageOn ) \
			printf("%sINFO reading error (more bytes then requested)\r\n", COL_RED); 

        // if 3 bytes are read put them into the data structure
        if (values_pos == 3)
		{
			pthread_mutex_lock(&parameter->data_protect);            

			/* Does the input value belong to the valid range? */
            if (valueCheck(parameter) == false)
			{
				pthread_mutex_unlock(&parameter->data_protect);
				if(synch(parameter) == false)
				{
				    pthread_mutex_unlock(&parameter->setup_protect);
					return false;
				}
			}
			else
			{ 			
				// signal that the data was updated
				parameter->dataUpdated = true;
				pthread_mutex_unlock  (&parameter->data_protect);
				pthread_cond_broadcast(&parameter->data_updated);
			}

            values_pos = 0;
			pthread_mutex_unlock(&parameter->setup_protect);
            razorSleep(40);
			pthread_mutex_lock(&parameter->setup_protect);
        }
    }

    pthread_mutex_unlock(&parameter->setup_protect);
    return true;
}

/*----------------------------------------------------------------------------------------------------*/

bool readOnRequest(struct thread_parameter *parameter)
{
    char singleByte = 'D';
    int result = 0;
    int bufferInput = 0;
    size_t values_pos = 0;

	// ensure that currently only this function changes razor setup
	pthread_mutex_lock(&parameter->setup_protect);

	// disable output continuous stream
    write(parameter->setup->tty_fd, "#o0", 3);
    razorSleep(20);
	
	// define output mode
	write(parameter->setup->tty_fd, "#ob", 3);
    razorSleep(20);

    tcflush(parameter->setup->tty_fd, TCIFLUSH);

	if(synch(parameter) == false)
	{
		pthread_mutex_unlock(&parameter->setup_protect);		
		return false;
	}

    /* variables to store time values 
     * used to measure how long requesting takes */
    struct timeval t0, t1, t2;

	while(parameter->razor_is_running)
	{
		while( !(parameter->data->dataRequest) )
			pthread_cond_wait(&parameter->update, &parameter->setup_protect);

		// try to send request
		if(write(parameter->setup->tty_fd, "#f", 2) != 2)
		{
			// ... in case of failed sending
			printf("%sINFO: unable to send request\n\r", COL_RED);
			pthread_mutex_lock(&parameter->data_protect);
			parameter->data->dataRequest = false;
			pthread_mutex_unlock(&parameter->data_protect);
		}

	    gettimeofday(&t0, NULL);
	    t1 = t0;

        while (parameter->data->dataRequest)
		{
        	result = read(parameter->setup->tty_fd, &singleByte, 1);

			#if debug
            	printf("%sInside buffer: %d \t read bytes: %d\n\r", COL_BLUE, bufferInput, result);
			#endif

            if (result == 1)
			{
				// ensure that currently only this function changes razor data
				pthread_mutex_lock(&parameter->data_protect);
		        parameter->data->buffer.ch[bufferInput] = singleByte;
		        bufferInput++;

		        if (bufferInput == 4)
				{
		        	
                	parameter->data->values[values_pos] = parameter->data->buffer.f;
                	values_pos++;
                	bufferInput = 0;
		        }
				pthread_mutex_unlock(&parameter->data_protect);
            }
			else if((result > 1) && parameter->setup->messageOn)\
				printf("%sINFO: reading error (more bytes then requested)\r\n", COL_RED); 

            // if new data is available on the serial port, print it out
            if (values_pos == 3)
			{
				pthread_mutex_lock(&parameter->data_protect);            

				/* Does the input value belong to the valid range? */
		        if (valueCheck(parameter) == false)
				{
					pthread_mutex_unlock(&parameter->data_protect);
					if(synch(parameter) == false)
					{
						pthread_mutex_unlock(&parameter->setup_protect);
						return false;
					}
				}
				else
				{ 			
					// signal that the data was updated
					parameter->dataUpdated = true;
					parameter->data->dataRequest = false;
					pthread_mutex_unlock  (&parameter->data_protect);
					pthread_cond_broadcast(&parameter->data_updated);
				}

		        values_pos = 0;
				pthread_mutex_unlock(&parameter->setup_protect);
		        razorSleep(20);
				pthread_mutex_lock(&parameter->setup_protect);
			}

            gettimeofday(&t2, NULL);

            if (elapsed_ms(t1, t2) > 200)
			{
            	// 200ms elapsed since last request and no answer -> request synch again
                // (this happens when DTR is connected and Razor resets on connect)
                write(parameter->setup->tty_fd, "#f", 2);
                values_pos = 0;
                bufferInput = 0;
                t1 = t2;
			}

            // Time out? 
            if (elapsed_ms(t0, t2) > connect_timeout_ms)
			{
				// TIME OUT!
                if (parameter->setup->messageOn) printf("%sINFO: request failed. (time out)\n\r", COL_RED);
				parameter->data->dataRequest = false;
            }
		}
	}

    pthread_mutex_unlock(&parameter->setup_protect);
    return true;
}

/*----------------------------------------------------------------------------------------------------*/

void* readingRazor ( struct thread_parameter *parameter )
{
	pthread_mutex_lock(&parameter->setup_protect);
    if (parameter->setup->streaming_Mode == STREAMINGMODE_CONTINUOUS)
	{
		pthread_mutex_unlock(&parameter->setup_protect);
        readContinuously(parameter);
    }
	else if (parameter->setup->streaming_Mode == STREAMINGMODE_ONREQUEST)
	{
		pthread_mutex_unlock(&parameter->setup_protect);
		readOnRequest(parameter);
    }
	else
	{
		pthread_mutex_unlock(&parameter->setup_protect);
        printf("%sINFO: No streaming mode selected!", COL_YELLOW);
    }

    razorAHRS_quit(parameter);
    pthread_exit(NULL);
}

/*----------------------------------------------------------------------------------------------------*/


/* mode: 0 -> single (frame on request)
 *       1 -> continuous
 *
 * format: 0 -> integer (4 Byte per frame) 
 *         1 -> floating point (12 Byte per frame)
 */
struct thread_parameter* razorAHRS ( speed_t baudRate, char* port, int mode, int format )
{
	if(port == NULL)
	{
		printf("%sNo Port selected. Please define port: e.g. /dev/ttyUSB0\r\n", COL_RED);
		return NULL;
	}	

	struct thread_parameter *parameter;
	parameter        = (struct thread_parameter*) calloc(1, sizeof(struct thread_parameter));
    parameter->setup = (struct razorSetup*)       calloc(1, sizeof (struct razorSetup));
 	parameter->data  = (struct razorData*)        calloc(1, sizeof (struct razorData));

	pthread_mutex_init(&parameter->setup_protect, NULL);
	pthread_mutex_init(&parameter->data_protect, NULL);
	pthread_cond_init (&parameter->data_updated, NULL);
	pthread_cond_init (&parameter->update, NULL);

    // saving port id and name in the setup
    parameter->setup->tty_fd           = open(port, O_RDWR | O_NONBLOCK);
    parameter->setup->baudRate         = baudRate;
    parameter->setup->port             = port;
    parameter->setup->streaming_Mode   = (mode == 1)   ? STREAMINGMODE_ONREQUEST : STREAMINGMODE_CONTINUOUS;
    parameter->setup->streaming_Format = STREAMINGFORMAT_BINARY_FLOAT;
    parameter->setup->messageOn        = message_on;
	parameter->dataUpdated             = false;

	return parameter;
}

/*----------------------------------------------------------------------------------------------------*/


int razorAHRS_start ( struct thread_parameter *parameter)
{

	pthread_mutex_lock(&parameter->setup_protect);

    //saving current termios configurations of tty_fd
    if (tcgetattr(parameter->setup->tty_fd, &parameter->setup->old_tio) != 0)
	{
        parameter->setup->tio_config_changed = false;
        printf("%sINFO: Saving configuration of %s failed.\n\r \
				--> tcgetattr(fd, &old_tio)\r\n", COL_RED, parameter->setup->port);

        /*reactivating the previous configurations of STDOUT_FILENO 
        because of breaking the process */
        resetConfig(parameter->setup);

        return -1;
    }

    struct termios tio;
    memset(&tio, 0, sizeof (tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 10;
    cfsetospeed(&tio, parameter->setup->baudRate); // 57600 baud
    cfsetispeed(&tio, parameter->setup->baudRate); // 57600 baud
    tcsetattr(parameter->setup->tty_fd, TCSANOW, &tio);
    parameter->setup->tio_config_changed = true;

	pthread_mutex_unlock(&parameter->setup_protect);

	parameter->razor_is_running = true;

    parameter->thread_id = pthread_create(&parameter->thread, NULL, (void*) &readingRazor, parameter);

    return 0;
}

/*----------------------------------------------------------------------------------------------------*/

int razorAHRS_quit ( struct thread_parameter *parameter )
{	
	bool quit_error = false;

    // reactivating previous configurations of tty_fd and the STDOUT_FILENO
	pthread_mutex_lock(&parameter->setup_protect);    

    // reactivate text mode
    write(parameter->setup->tty_fd, "#ot", 3);

    resetConfig(parameter->setup);
    pthread_mutex_unlock(&parameter->setup_protect);

    quit_error = (pthread_mutex_destroy(&parameter->setup_protect) != 0) ? true : false;
    quit_error = (pthread_mutex_destroy(&parameter->data_protect)  != 0) ? true : false;
	quit_error = (pthread_cond_destroy (&parameter->data_updated)  != 0) ? true : false;

    free(parameter->setup);
    free(parameter->data);
    return (quit_error) ? -1 : 0;
}

/*----------------------------------------------------------------------------------------------------*/

/*  stops the razor thread
 */
void razorAHRS_stop ( struct thread_parameter *parameter )
{
	parameter->razor_is_running = false;
}

/*----------------------------------------------------------------------------------------------------*/

/*  requests a single data frame
 */
int razorAHRS_request ( struct thread_parameter *parameter )
{

	if(parameter->setup->streaming_Mode == STREAMINGMODE_ONREQUEST)
	{
		pthread_mutex_lock(&(parameter->data_protect));	
		parameter->data->dataRequest = true;
		pthread_mutex_unlock  (&(parameter->data_protect));
		pthread_cond_broadcast(&(parameter->update));
		return 0;
	}

	return -1;
}

/*----------------------------------------------------------------------------------------------------*/

// output read values on stdout
void* razorPrinter ( void* args )
{
    struct thread_parameter* parameter = (struct thread_parameter*) args;

	while( (parameter->razor_is_running) && (parameter->printer_is_running) )
	{
		pthread_mutex_lock(&parameter->data_protect);
		while( !parameter->dataUpdated ) 
			pthread_cond_wait(&parameter->data_updated, &parameter->data_protect);
		pthread_mutex_unlock(&parameter->data_protect);

		pthread_mutex_lock(&parameter->data_protect);
        printf("%sYAW = %6.1f \t %sPITCH = %6.1f \t %sROLL = %6.1f \r\n", COL_NORMAL, \
				parameter->data->values[0], COL_YELLOW, parameter->data->values[1], COL_CYAN, parameter->data->values[2]);
		if(parameter->data->data_fail == true) printf("%sDATA FAILURE -- INVALID VALUES\r\n", COL_RED);	
		parameter->dataUpdated = false;
		pthread_mutex_unlock(&parameter->data_protect);
	}
	
    pthread_exit(NULL);
}

/*----------------------------------------------------------------------------------------------------*/

void razorPrinter_start ( struct thread_parameter *parameter, pthread_t *printer )
{
	parameter->printer_is_running = true;
    pthread_create(printer, NULL, (void*) &razorPrinter, parameter);
}

/*----------------------------------------------------------------------------------------------------*/

int razorPrinter_stop ( struct thread_parameter *parameter)
{
    parameter->printer_is_running = false;
    return 0;
}

#endif // RAZORAHRS_C
