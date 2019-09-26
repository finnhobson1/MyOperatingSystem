/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "P6.h"

void pipeRead(int id, int pid) {
  int fd;
  fd = open(id, pid);
  char *message = read(fd);
  if (message == "Hi!") write( STDOUT_FILENO, "Message Received\n", 18 );
}

void pipeWrite(int id, int pid) {
  int fd;
  char *message = "Hi!";
  fd = open(id, pid);
  write( fd, message, 3 );
}

void main_P6() {
  write( STDOUT_FILENO, "P6 ", 3 );
  int parentPID = 1;
  int childPID;
  int pipeID = 1;

  int pid = fork();
  if (pid > 0) {
    childPID = pid;
    mkfifo(pipeID, parentPID, childPID);
  }
  else if (pid == 0) pipeRead(pipeID, childPID);
  pipeWrite(pipeID, parentPID);
  exit( EXIT_SUCCESS );
}
