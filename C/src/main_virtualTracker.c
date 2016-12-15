#include "virtualTracker.h"

int main (int argc, char *argv[]) {
  if (argc != 2) {
    printf ("The path to the socket is required.\n");
    return -1;
  }
  if (virtualTracker (20, B57600, argv[1]) != 0) {
    printf ("Failed to start the virtual tracker on socket %s.\n", argv[1]);
    return -1;
  }
  return 0;
}
