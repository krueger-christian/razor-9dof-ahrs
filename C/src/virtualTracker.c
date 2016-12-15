#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include <fcntl.h>
#include <termios.h>

#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <errno.h>

#include "virtualTracker.h"


/** 
Computes the elapsed time in ms.
*/
unsigned long elapsed_ms (struct timeval start, struct timeval end) {
  return (long) ((end.tv_sec - start.tv_sec) * 1000 + (end.tv_usec - start.tv_usec) / 1000);
}

bool virtualTracker (unsigned int frequency, speed_t baudRate, char *port) {

  //Setup socket
  struct vt_adjustment setting;
  memset (&setting, 0, sizeof (setting));
  setting.vt_port = port;
  setting.vt_baudRate = baudRate;
  setting.vt_frequency = frequency;
  setting.streamingformat = STREAMINGFORMAT_ASCII;
  setting.output_Mode = STREAMINGMODE_CONTINOUS;
  setting.waitingTime = 1000 / frequency;

  struct termios stdio;
  memset (&stdio, 0, sizeof (stdio));
  stdio.c_iflag = 0;
  stdio.c_oflag = 0;
  stdio.c_cflag = 0;
  stdio.c_lflag = 0;
  stdio.c_cc[VMIN] = 1;
  stdio.c_cc[VTIME] = 10;
  tcsetattr (STDOUT_FILENO, TCSANOW, &stdio);
  tcsetattr (STDOUT_FILENO, TCSAFLUSH, &stdio);
  fcntl (STDIN_FILENO, F_SETFL, O_NONBLOCK);    // make the reads non-blocking

  setting.tty_fd = open (setting.vt_port, O_RDWR | O_NONBLOCK);
  if (setting.tty_fd == -1) {
    printf("Socket %s could not be opened: errno %i\n.", setting.vt_port, errno);
    return -1;
  }

  struct termios tio;
  memset (&tio, 0, sizeof (tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL;   // 8n1, see termios.h for more information
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 10;
  cfsetospeed (&tio, setting.vt_baudRate);
  cfsetispeed (&tio, setting.vt_baudRate);
  tcsetattr (setting.tty_fd, TCSANOW, &tio);

  printf ("Connecting to: %s (%d) \r\n", setting.vt_port, setting.tty_fd);

  float data[3] = {0.0, 0.0, 0.0};
  char console = 'c';
  while (true) {
    bool request = false;

    //Receiving
    unsigned char read_byte[3];
    unsigned int read_byte_length;

    read_byte_length = read (setting.tty_fd, &read_byte, 1);

    if (read_byte_length == 1 && read_byte[0] == '#') { //Sync?
      read_byte_length = read (setting.tty_fd, &read_byte, 1);  //Accept: {'s', '0'}

      if (read_byte_length == 1 && read_byte[0] == 's') {
        read_byte_length = read (setting.tty_fd, &read_byte, 2);  //Accept: {id1, id2}
        if (read_byte_length == 2) {
          unsigned char sync_message[] = { '#', 'S', 'Y', 'N', 'C', 'H', read_byte[0], read_byte[1], '\r', '\n' };
          printf ("Replying to sync request: %s", sync_message);

          write (setting.tty_fd, sync_message, sizeof (sync_message) / sizeof (sync_message[0]));
        }
        continue;
      }
    

      if (read_byte_length == 1 && read_byte[0] == 'o') { //Command?
        read_byte_length = read (setting.tty_fd, &read_byte, 1);

        if (read_byte_length == 1) {
          switch (read_byte[0]) {
          case 'b':
            printf("Sending binary.\n");
            setting.streamingformat = STREAMINGFORMAT_BINARY;
            break;
          case 't':
            printf("Sending ASCII.\n");
            setting.streamingformat = STREAMINGFORMAT_ASCII;
            break;
          case '1':
            printf("Sending continously.\n");
            setting.output_Mode = STREAMINGMODE_CONTINOUS;
            break;
          case '0':
            printf("Sending on request.\n");
            setting.output_Mode = STREAMINGMODE_ONREQUEST;
            break;
          case 'f':
            request = true;
            break;
          }
        }
		continue;
      }

	  if (read_byte_length == 1 && read_byte[0] == 'f') { //Command?
        request = true;
      }
    }

    //Sending
    if (setting.output_Mode == STREAMINGMODE_CONTINOUS || request == true) {
	  data[0] = (data[0] <= 360) ? (data[0] + 1) : 0;
      data[1] = (data[1] <= 360) ? (data[1] + 1) : 0;
      data[2] = (data[2] <= 360) ? (data[2] + 1) : 0;


      ssize_t len = snprintf (NULL, 0, "#YPR=%06.2f,%06.2f,%06.2f\n", data[0], data[1], data[2]);
      char output[len];
      sprintf (output, "#YPR=%06.2f,%06.2f,%06.2f", data[0], data[1], data[2]);
      printf("%s\r\n", output);

      ssize_t write_result;
      switch (setting.streamingformat) {
      case STREAMINGFORMAT_BINARY:
        write_result = write (setting.tty_fd, data, 3 * sizeof (float));
        break;
      case STREAMINGFORMAT_ASCII:
        write_result = write (setting.tty_fd, output, len);
        break;
      }
      
      if (write_result <= 0) {
         printf("Write to socket failed with result %i; errno %i", write_result, errno);
      }
      request = false;
    }
    usleep (setting.waitingTime * 1000);
  }

  close (setting.tty_fd);

  return true;
}
