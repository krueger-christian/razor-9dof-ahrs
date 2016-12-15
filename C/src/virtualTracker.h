#ifndef VIRTUALTRACKER_H
#define VIRTUALTRACKER_H

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

enum STREAMINGMODE { STREAMINGMODE_ONREQUEST, STREAMINGMODE_CONTINOUS };
enum STREAMINGFORMAT { STREAMINGFORMAT_ASCII, STREAMINGFORMAT_BINARY };

struct vt_adjustment
{
  struct termios old_tio;
  struct termios old_stdio;

  enum STREAMINGFORMAT streamingformat;
  enum STREAMINGMODE output_Mode;

  char *vt_port;
  int tty_fd;
  int vt_frequency;
  unsigned int waitingTime;     //in ms
  speed_t vt_baudRate;
};

// Computes the elapsed time in ms.
unsigned long elapsed_ms (struct timeval start, struct timeval end);

bool virtualTracker (unsigned int frequency, speed_t baudRate, char *port);

#endif;
