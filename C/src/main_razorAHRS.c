/*****************************************************************
 *                                                               *
 *                                                               *
 *               -- DEMONSTRATION PROGRAMM --                    *
 *                                                               *
 *                                                               *
  *                                                               *
 *     C-coded functions to read the the measuring data of       *
 *     the 9 Degree of Measurement Attitude and Heading          *
 *     Reference System of Sparkfun's "9DOF Razor IMU"           *
 *     and "9DOF Sensor Stick"                                   *
 *                                                               *
 *     a former version, used as reference and coded in C++      *
 *     was written by Peter Bartz:                               *
 *     https://github.com/ptrbrtz/razor-9dof-ahrs                *
 *                                                               *
 *     Quality & Usability Lab, TU Berlin                        *
 *     & Deutsche Telekom Laboratories                           *
 *     Christian Krüger                                          *
 *     2016                                                      * 
 *                                                               *
 *     further informations:                                     *
 *     https://github.com/krueger-christian/razorAHRS            *
 *                                                               *
 ****************************************************************/

#include "razorAHRS.h"

/*-----------------------------------------------------------------*/

int main(int argc, char* argv[])
{
    printf("_________________________________________________\r\n");
    printf("\n    RAZOR AHRS – Headtracker Reader\n");
    printf("_________________________________________________\r\n\n");

    printf("_________________________________________________\r\n");
    printf("\n  !\n\r    CHOOSE RUNNING MODE\n\r");
    printf("       PRESS     C    FOR CONTINUOUS OUTPUT\n\r");
    printf("       PRESS     S    FOR SINGLE FRAME OUTPUT\n\r");
    printf("       PRESS     Q    QUIT\n\r");
	printf("\n\r");
    printf("       (CONFIRM WITH ENTER)\n\n\n\r");

    char console = 'D';
    int mode;
	int format;

    while (1) 
	{
        // if new data is available on the console...
        scanf("%c", &console);

        // quit
        if ((console == 'q') || (console == 'Q')) return -1;

		// mode: continuous streaming
        if ((console == 'c') || (console == 'C'))
		{
            mode = 0;
            break;
        }
        // mode: read one frame per request
        if ((console == 's') || (console == 'S')) 
		{
            mode = 1;
            break;
        } 
        printf("\n  !\n\r    INVALID INPUT\n\n\r");
    }


	char* string;
	string = calloc(50, sizeof(char));

	if(argc < 2){
		printf("\n  !\n\r    NAME PORTNAME, e.g. /dev/ttyUSB0\n\r");
		printf("\n\r");
		printf("       (CONFIRM WITH ENTER)\n\n\n\r");

	    // if new data is available on the console...
		scanf("%s", string);
	}



	/* creating a management structure containing
     * a setting structure and a data structure as well as
	 * mutexes to ensure thread save use
     */ 
    struct thread_parameter* parameter = razorAHRS(B57600, argv[1], mode, 1);

	// starting the tracker
	if(( parameter != NULL ) && (razorAHRS_start(parameter) < 0 )) return -1;
	
	// starting a printer to output the data on standart output
	pthread_t printer;
	razorPrinter_start(parameter, &printer);

	console = 'D';

	if(mode == 0)
	{
		while(parameter->razor_is_running)
		{
		    if (read(STDIN_FILENO, &console, 1) > 0)
		      	if (console == '\n') razorAHRS_stop(parameter);
			razorSleep(20);
		}

		pthread_join(parameter->thread, NULL);
	}
	else if(mode == 1)
	{
		printf("\n\n\r");
		printf("    PRESS      ENTER  FOR SINGLE FRAME OUTPUT\n\r");
		printf("    PRESS  Q + ENTER  TO QUIT\n\n\r");

		while( parameter->razor_is_running )
		{
		    if (read(STDIN_FILENO, &console, 1) > 0)
			{
		      	if ((console == 'q' ) || (console == 'Q') )razorAHRS_stop(parameter);
				if  (console == '\n') razorAHRS_request(parameter);
		    }
			razorSleep(20);
		}
	}


    printf("\n_________________________________________________\r\n");
    return 0;
}
