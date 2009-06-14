#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#define sec(t) 0.5*(t.tv_sec+t.tv_usec*1e-6)

int quit = 0;

void signaled(int signal) {
  quit = 1;
}

int serial_read(int fd, unsigned char* data, ssize_t num) {
  ssize_t num_read = 0;
  struct timeval time;
  fd_set set;
  int error;

  while (num_read < num) {
    time.tv_sec = 0;
    time.tv_usec = 5000;

    FD_ZERO(&set);
    FD_SET(fd, &set);

    error = select(fd+1, &set, NULL, NULL, &time);
    if (error == 0)
      return -1;

    ssize_t n;
    n = read(fd, &data[num_read], num-num_read);
    if ((n < 0) && (errno != EWOULDBLOCK))
      return -1;
    if (n > 0)
      num_read += n;
  }
  
  return num_read;
}

int main( int argc, char * argv[] ) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s DEV\n", argv[0]);
    return -1;
  }

  int result = 0;
  int fd = open(argv[1], O_RDONLY | O_NDELAY);

  if (fd > 0) {
    struct termios tio;
    memset(&tio, 0, sizeof(struct termios));
    tio.c_cflag |= B500000;
    tio.c_cflag |= CS8;
    tio.c_cflag |= CSTOPB;

    if (!tcsetattr(fd, TCSANOW, &tio)) {
      ssize_t num_trans = 0, num_trans_rate = 0;
      double trans_rate = 0.0;
      struct timeval start_time, current_time;
      gettimeofday(&start_time, 0);
      
      signal(SIGINT, signaled);
      
      while (!quit) {
        ssize_t num;
        unsigned char data[64];
        memset(data, 0, sizeof(data));

        if ((num = serial_read(fd, data, sizeof(data))) > 0) {
          num_trans += num;
          gettimeofday(&current_time, 0);

          double dt = sec(current_time)-sec(start_time);
          if (dt >= 1.0) {
            trans_rate = ((num_trans-num_trans_rate)*8)/dt;

            gettimeofday(&start_time, 0);
            num_trans_rate = num_trans;
          }

          fprintf(stderr, "\rreceived %6d kB at %6d kBit/s", 
            num_trans/1024, (int)trans_rate);
        }
        else {
          fprintf(stderr, "read error\n");
          result = -1;
          break;
        }
      }
    }
    else {
      fprintf(stderr, "error setting device attributes\n");
      result = -1;
    }
    fprintf(stderr, "\n");

    close(fd);
  }
  else {
    fprintf(stderr, "error opening device\n");
    result = -1;
  }

  return result;
}
