#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <signal.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/uio.h>

#include "dns320l.h"

#define FAN_POLL_TIME         15
#define GPIO_POLL_TIME        1
#define SERVER_PORT           57367
#define SYSFS_GPIO_DIR        "/sys/class/gpio"

int ls;
int fd;

int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd, len;
	char buf[100];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}


void cleanup(int shut,int s,int howmany)
{
  int     retval;

  /*
   * Shutdown and close sock1 completely.
   */
  if (shut)
  {
      retval = shutdown(s,howmany);
      if (retval == -1)
          perror ("shutdown");
  }
  retval = close (s);
  if (retval)
      perror ("close");
} 

void sighandler(int sig)
{
    if (sig == SIGINT){
        cleanup(0, ls,1);
        exit(EXIT_SUCCESS);
    }
}

void declsighandler()
{
    struct sigaction action;

    sigemptyset(&action.sa_mask);
    sigaddset(&action.sa_mask,SIGINT);
    action.sa_flags = 0;
    action.sa_handler = sighandler;
    sigaction(SIGINT,&action,NULL);
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

int CheckResponse(char *buf, const char *cmd, int len)
{
    int i;
    int tmp;
    
    for(i=0;i<5;i++)
    {
        if(buf[i] != cmd[i])
        {
            printf("Char %i is %i but should be %i\n", i, buf[i], cmd[i]);
            return ERR_WRONG_ANSWER;
        }
    }
    if(buf[len-1] != cmd[len-1])
        return ERR_WRONG_ANSWER;
    tmp = (unsigned char)buf[5];
    return tmp;
}

int SendCommand(int fd, const char *cmd, int checkAnswer)
{
    int n;
    int i;
    unsigned int tmp;

    char buf[15]; // We need to keep the DateAndTime values here
    // Yes, we're sending byte by byte here - b/c the lenght of 
    // commands and responses can vary!
    
    i=0;
    do
    {
        write(fd, &cmd[i], 1);
        i++;
    } while(cmd[i-1] != CMD_STOP_MAGIC);
    
    i=0;
    do
    {
        n = read(fd, &buf[i], 1);
        i++;
    } while((n == 1) && (buf[i-1] != CMD_STOP_MAGIC));
    

    if(buf[i-1] != CMD_STOP_MAGIC)
    {
        printf("Got no stop magic!\n");
        return ERR_WRONG_ANSWER;
    }
    else
    {
        if(checkAnswer)
        {
            tmp = CheckResponse(buf, cmd, i);
            usleep(20000); // Give the µC some time to answer...

            i=0;
            do
            {
                n = read(fd, &buf[i], 1);
                i++;
            } while((n == 1) && (buf[i-1] != CMD_STOP_MAGIC));
            

            if(buf[i-1] != CMD_STOP_MAGIC)
            {
                printf("Got no stop magic!\n");
                return ERR_WRONG_ANSWER;
            }

            CheckResponse(buf, AckFromSerial, i);
            return tmp;
        }
        else
        {
            return CheckResponse(buf, AckFromSerial, i);
        }
    }
}

int HandleCommand(char *message, int messageLen, char *retMessage, int bufSize)
{
    int tmp;
    int len;
    int i;
    char cmp[] = "DeviceReady";
    printf("Handling Command: %s\n", message);

    if(strncmp(message, "DeviceReady", messageLen) == 0)
    {
      printf("DeviceReady\n");
      if(SendCommand(fd, DeviceReadyCmd, 0) == 0)
          strncpy(retMessage, "OK\n", bufSize);
      else
          strncpy(retMessage, "ERR\n", bufSize);
    }
    else if(strncmp(message, "GetTemperature", messageLen) == 0)
    {
      printf("GetTemperature\n");
      tmp = SendCommand(fd, ThermalStatusGetCmd, 1);
      tmp = ThermalTable[tmp];
      sprintf(retMessage, "%d", tmp);
      len = strlen(retMessage);
      retMessage[len] = '\n';
      retMessage[len+1] = '\0';
    }
    else if(strncmp(message, "DeviceShutdown", strlen("DeviceShutdown")) == 0)
    {
      printf("DeviceShutdown");
      if(messageLen >= (strlen("DeviceShutdown") + 2))
      {
        //tmp = atoi(&message[strlen("DeviceShutdown") + 1]); // FIXME: The parameter is never passed, we default to 10s here..
        //printf("%s\n", tmp);
        if(SendCommand(fd, DeviceShutdownCmd, 0) == 0)
          strncpy(retMessage, "OK\n", bufSize);
        else
          strncpy(retMessage, "ERR\n", bufSize);
      }
    }
    else if(strncmp(message, "quit", messageLen) == 0)
    {
      printf("Quit\n");
      strncpy(retMessage, "Bye\n", bufSize);
      return 1;
    }
    else
    {
      strncpy(retMessage, "Command not Understood!\n", bufSize);
    }
    
    
    //strcpy(retMessage, "OK\n");
    
    return 0;
}

int main(int args, char *argv[])
{

    char *portname = "/dev/ttyS1"; // We hardcode the path, as this daemon is inteded to run on one box only
    char response[100];
    int n;
    int i;
    int j;
    int powerBtn;
    int pressed;
    int opt;
    int sleepCount;
    int pollTimeMs;
    char buf[100];
    int temperature;
    int fanSpeed;
    struct sockaddr_in s_name;
    struct pollfd *fds = NULL;
    nfds_t nfds;
    int retval;
    int ret;
    int atmark;
    int msgIdx;
    char message[100];
    socklen_t namelength;
    pressed = 0;
    nfds = 1;
    opt = 1;
    sleepCount = 0;
    pollTimeMs = 10; // Sleep 10ms for every loop
    fanSpeed = -1;
    
    if ((ls = socket (AF_INET, SOCK_STREAM, 0)) == -1){
         perror( "socket");
         exit(EXIT_FAILURE);
    }

    if (setsockopt(ls,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof opt)<0){
        printf ("setsockopt (SO_RESUSEADDR): %s\r\n",strerror(errno));
        exit(EXIT_FAILURE);
    }


    s_name.sin_family = AF_INET;
    s_name.sin_port = htons(SERVER_PORT);
    s_name.sin_addr.s_addr = htonl(INADDR_ANY);

    printf(" \t Bind name to ls. \n");
    retval = bind (ls,(struct sockaddr *)&s_name, sizeof s_name);
    if (retval)
    {
        perror("bind");
        cleanup(0, ls,1);
        exit(EXIT_FAILURE);
    }

    printf(" \t Listen on ls for connections. \n");
    retval = listen (ls, 5);
    if (retval)
    {
        perror("listen");
        cleanup(0, ls,1);
        exit(EXIT_FAILURE);
    }
    declsighandler();
    

    fds = (struct pollfd *)calloc(1,nfds*sizeof(struct pollfd));
    fds->fd = ls;
    fds->events = POLLIN | POLLPRI;
 
    
    
    
    
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
            return;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking
    
    // Flush the serial port first...
    
    read(fd, buf, 100);
    
    
    if(SendCommand(fd, DeviceReadyCmd, 0) == 0)
        printf("DeviceReady command sent, going to Fan Control Mode...\n");
    else
    {
        printf("Error sending DeviceReady command, exit!\n");
        return 1;
    }

    while(1)
    {
        sleepCount = 0;
        temperature = SendCommand(fd, ThermalStatusGetCmd, 1);
        if(temperature > 0)
        {
            temperature = ThermalTable[temperature];
            printf("Read Temperature: %i\n", temperature);
            if(temperature < (TEMP_LOW - HYSTERESIS))
            {
                if(fanSpeed != 0)
                {
                    printf("Set Fan Stop\n");
                    SendCommand(fd, FanStopCmd, 0);
                    fanSpeed = 0;
                }
            }
            else if(temperature < TEMP_LOW)
            {
                if(fanSpeed > 1)
                {
                    printf("Set Fan Half\n");
                    SendCommand(fd, FanHalfCmd, 0);
                    fanSpeed = 1;
                }
            }
            else if(temperature < (TEMP_HIGH - HYSTERESIS))
            {
                if(fanSpeed != 1)
                {
                    printf("Set Fan Half\n");
                    SendCommand(fd, FanHalfCmd, 0);
                    fanSpeed = 1;
                }
            }
            else if(temperature < TEMP_HIGH)
            {
                if(fanSpeed < 1)
                {
                    printf("Set Fan Half\n");
                    SendCommand(fd, FanHalfCmd, 0);
                    fanSpeed = 1;
                }
            }
            else
            {
                if(fanSpeed != 2)
                {
                    printf("Set Fan Full\n");
                    SendCommand(fd, FanFullCmd, 0);
                    fanSpeed = 2;
                }
            }
        }
        else
        {
            printf("Error reading Temperature!\n");
        }
        
        
        while((sleepCount  * pollTimeMs) < (FAN_POLL_TIME * 1000))
        {
            if(((sleepCount * pollTimeMs) % (GPIO_POLL_TIME * 1000)) == 0)
            {
                if(gpio_get_value(GPIO_BUTTON_POWER, &powerBtn) == 0)
                {
                    if((powerBtn == 0) && !pressed)
                    {
                        pressed = 1;
                        printf("Power Button Pressed!\n");
                    }
                }
            
            }
            sleepCount++;

            ret=poll(fds,nfds,pollTimeMs); // Time out after pollTimeMs
            if (ret == -1){
              perror ("poll");
              exit(EXIT_FAILURE);
            }
            for (i=0;(i<nfds) && (ret);i++)
            {
               if (!(fds+i)->revents)
                   continue;
               printf("  after : revents=0x%x, ret=%d\n\n",
                    (fds+i)->revents,
                    ret);
               ret--;
               if (((fds+i)->fd == ls) && ((fds+i)->revents & POLLIN))
               {
                    /*
                     * Accept connection from socket ls:
                     * accepted connection will be on socket (fds+nfds)->fd.
                     */
                   printf(" \t POLLIN on ls. Accepting connection\n");
                   namelength = sizeof (s_name);
                   fds = (struct pollfd *)realloc(fds,(nfds+1)*sizeof(struct pollfd));
                   (fds+nfds)->fd  = accept (ls, (struct sockaddr *)&s_name, &namelength);
                   if ((fds+nfds)->fd == -1)
                   {
                      perror ("accept");
                      cleanup(0, (fds+nfds)->fd, 1);
                      fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
                      continue;
                   }
                   (fds+nfds)->events = POLLIN | POLLRDNORM;
                   nfds++;
                   continue;
               }
               if ((fds+i)->revents & POLLNVAL)
               {
                   printf ("POLLNVAL on socket. Freeing resource\n");
                   nfds--;
                   memcpy(fds+i,fds+i+1,nfds-i);
                   fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
                   continue;
               }
               if ((fds+i)->revents & POLLHUP)
               {
                   printf ("\t POLLHUP => peer reset connection ...\n");
                   cleanup(0,(fds+i)->fd,2);
                   nfds--;
                   memcpy(fds+i,fds+i+1,nfds-i);
                   fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
                   continue;
               }
              if ((fds+i)->revents & POLLERR){
                  printf ("\t POLLERR => peer reset connection ...\n");
                  cleanup(0,(fds+i)->fd,2);
                  nfds--;
                  memcpy(fds+i,fds+i+1,nfds-i);
                  fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
                  continue;
             }
             if ((fds+i)->revents & POLLRDNORM)
             { 
                 retval = recv((fds+i)->fd, message, sizeof(message)-1, 0); // Don't forget the string terminator here!
                 printf(" \t -> (recv) retval = %d.\n",retval);  /* ped */
                 msgIdx = retval;
                 if (retval <=0)
                 {
                    if (retval == 0)
                    {
                       printf ("\t recv()==0 => peer disconnected...\n");
                       cleanup(1,(fds+i)->fd,2);
                    }
                    else 
                    {
                        perror ("\t receive");
                        cleanup( 0, (fds+i)->fd,1);
                    }
                    nfds--;
                    memcpy(fds+i,fds+i+1,nfds-i);
                    fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
                    continue;
                 }
                 while((retval > 0) && (message[msgIdx-2] != '\r') && ((msgIdx+1) < sizeof(message)))
                 {
                     retval = recv((fds+i)->fd, &message[msgIdx-2], sizeof(message) - retval - 1, 0);
                     printf(" \t -> (recv) retval = %d.\n", retval);                     
                     if(retval > 0)
                         msgIdx += retval - 2;
                 }
                 if(msgIdx > 1)
                   if(message[msgIdx-1] == '\n')
                     if(message[msgIdx-2] == '\r')
                       message[msgIdx-2] = '\0';
                     else
                       message[msgIdx-1] = '\0';

                 printf (" \t Normal message :  %.*s\n",retval,message);
                 msgIdx = HandleCommand(message, msgIdx, response, sizeof(response));
                 retval = send((fds+i)->fd, response, strlen(response), 0);
                 if((retval < 0) || (msgIdx == 1))
                 {
                     printf("\t send()==0 => peer disconnected...\n");
                     cleanup(1,(fds+1)->fd, 2);
                 }
                 continue;
             }
            }       
        }
    }
    
    return 0;
}
