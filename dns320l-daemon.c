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
#include <syslog.h>
#include <iniparser.h>

#include "dns320l.h"


int ls;
int fd;

int gpio_get_value(unsigned int gpio, unsigned int *value, char *gpioDir)
{
  int fd, len;
  char buf[100];
  char ch;

  len = snprintf(buf, sizeof(buf), "%s/gpio%d/value", gpioDir, gpio);

  fd = open(buf, O_RDONLY);
  if (fd < 0) {
    syslog(LOG_ERR, "gpio/get-value");
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
      syslog(LOG_ERR, "shutdown");
  }
  retval = close (s);
  if (retval)
    syslog(LOG_ERR, "close");
} 

static void sighandler(int sig)
{
  syslog(LOG_DEBUG, "Signal Handler called\n");
  switch(sig)
  {
  case SIGINT:
    cleanup(0, ls, 1);
    exit(EXIT_SUCCESS);
    break;
  case SIGTERM:
    cleanup(0, ls, 1);
    syslog(LOG_INFO, "Shutting down machine in 10s...\n");
    SendCommand(fd, DeviceShutdownCmd, 0);
    exit(EXIT_SUCCESS);
    break;
  }
}

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    syslog(LOG_ERR, "error %d from tcgetattr", errno);
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
    syslog(LOG_ERR, "error %d from tcsetattr", errno);
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
    syslog(LOG_ERR, "error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    syslog(LOG_ERR, "error %d setting term attributes", errno);
}

int CheckResponse(char *buf, const char *cmd, int len)
{
  int i;
  int tmp;

  for(i=0;i<5;i++)
  {
    if(buf[i] != cmd[i])
    {
      syslog(LOG_ERR, "Char %i is %i but should be %i\n", i, buf[i], cmd[i]);
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
    syslog(LOG_ERR, "Got no stop magic!\n");
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
        syslog(LOG_ERR, "Got no stop magic!\n");
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
  syslog(LOG_DEBUG, "Handling Command: %s\n", message);

  if(strncmp(message, "DeviceReady", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DeviceReady\n");
    if(SendCommand(fd, DeviceReadyCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "GetTemperature", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetTemperature\n");
    tmp = SendCommand(fd, ThermalStatusGetCmd, 1);
    tmp = ThermalTable[tmp];
    sprintf(retMessage, "%d", tmp);
    len = strlen(retMessage);
    retMessage[len] = '\n';
    retMessage[len+1] = '\0';
  }
  else if(strncmp(message, "DeviceShutdown", strlen("DeviceShutdown")) == 0)
  {
    syslog(LOG_DEBUG, "DeviceShutdown");
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
    syslog(LOG_DEBUG, "Quit\n");
    strncpy(retMessage, "Bye\n", bufSize);
    return 1;
  }
  else if(strncmp(message, "EnablePowerRecovery", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "EnablePowerRecovery");
    if(SendCommand(fd, APREnableCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "DisablePowerRecovery", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DisablePowerRecovery");
    if(SendCommand(fd, APRDisableCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "GetPowerRecoveryState", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetPowerRecoveryState");
    tmp = SendCommand(fd, APRStatusCmd, 1);
    sprintf(retMessage, "%d", tmp);
    len = strlen(retMessage);
    retMessage[len] = '\n';
    retMessage[len+1] = '\0';
  }
  else if(strncmp(message, "EnableWOL", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "EnableWOL");
    if(SendCommand(fd, WOLStatusEnableCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "DisableWOL", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DisableWOL");
    if(SendCommand(fd, WOLStatusDisableCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "GetWOLState", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetWOLState");
    tmp = SendCommand(fd, WOLStatusGetCmd, 1);
    sprintf(retMessage, "%d", tmp);
    len = strlen(retMessage);
    retMessage[len] = '\n';
    retMessage[len+1] = '\0';
  }
  else if(strncmp(message, "PowerLedOn", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedOn");
    if(SendCommand(fd, PwrLedOnCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "PowerLedOff", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedOff");
    if(SendCommand(fd, PwrLedOffCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "PowerLedBlink", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedBlink");
    if(SendCommand(fd, PwrLedBlinkCmd, 0) == 0)
      strncpy(retMessage, "OK\n", bufSize);
    else
      strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "systohc", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "systohc");
    strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "hctosys", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "hctosys");
    strncpy(retMessage, "ERR\n", bufSize);
  }
  else if(strncmp(message, "help", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "help");
    strncpy(retMessage, "Available Commands: DeviceReady, GetTemperature, DeviceShutdown, "
            "EnablePowerRecovery, DisablePowerRecovery, GetPowerRecoveryState, "
            "EnableWOL, DisableWOL, GetWOLState, PowerLedOn, "
            "PowerLedOff, PowerLedBlink, quit\n", bufSize);
  }
  else
  {
    strncpy(retMessage, "Command not Understood!\n", bufSize);
  }

  return 0;
}

int main(int args, char *argv[])
{

  char *portname; // We hardcode the path, as this daemon is inteded to run on one box only
  char *gpioDir;
  int serverPort;
  int fanPollTime;
  int gpioPollTime;
  char response[500];
  int i;
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
  int msgIdx;
  int tempLow;
  int tempHigh;
  int hysteresis;
  char message[500];
  dictionary *iniFile;
  socklen_t namelength;
  pressed = 0;
  nfds = 1;
  opt = 1;
  sleepCount = 0;
  pollTimeMs = 10; // Sleep 10ms for every loop
  fanSpeed = -1;

  signal(SIGTERM, sighandler);
  signal(SIGINT, sighandler);


  iniFile = iniparser_load("/etc/dns320l-daemon.ini");
  portname = iniparser_getstring(iniFile, "Serial:Port", "/dev/ttyS1");
  fanPollTime = iniparser_getint(iniFile, "Fan:PollTime", 15);
  tempLow = iniparser_getint(iniFile, "Fan:TempLow", 45);
  tempHigh = iniparser_getint(iniFile, "Fan:TempHigh", 50);
  hysteresis = iniparser_getint(iniFile, "Fan:Hysteresis", 2);
  gpioPollTime = iniparser_getint(iniFile, "GPIO:PollTime", 1);
  gpioDir = iniparser_getstring(iniFile, "GPIO:SysfsGpioDir", "/sys/class/gpio");
  serverPort = iniparser_getint(iniFile, "Daemon:ServerPort", 57367);


  setlogmask(LOG_UPTO(LOG_DEBUG));
  openlog("dns320l-daemon", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);

  if ((ls = socket (AF_INET, SOCK_STREAM, 0)) == -1){
    syslog(LOG_ERR, "socket");
    exit(EXIT_FAILURE);
  }

  if (setsockopt(ls,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof opt)<0){
    syslog(LOG_ERR, "setsockopt (SO_RESUSEADDR): %s\r\n",strerror(errno));
    exit(EXIT_FAILURE);
  }


  s_name.sin_family = AF_INET;
  s_name.sin_port = htons(serverPort);
  s_name.sin_addr.s_addr = htonl(INADDR_ANY);

  syslog(LOG_DEBUG, "Bind name to ls. \n");
  retval = bind (ls,(struct sockaddr *)&s_name, sizeof s_name);
  if (retval)
  {
    syslog(LOG_ERR, "bind");
    cleanup(0, ls,1);
    exit(EXIT_FAILURE);
  }

  syslog(LOG_DEBUG, "Listen on ls for connections. \n");
  retval = listen (ls, 5);
  if (retval)
  {
    syslog(LOG_ERR, "listen");
    cleanup(0, ls,1);
    exit(EXIT_FAILURE);
  }
  syslog(LOG_INFO, "Server startup success on port %i\n", serverPort);

  fds = (struct pollfd *)calloc(1,nfds*sizeof(struct pollfd));
  fds->fd = ls;
  fds->events = POLLIN | POLLPRI;

  fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    syslog(LOG_ERR, "error %d opening %s: %s", errno, portname, strerror (errno));
    return;
  }

  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking

  // Flush the serial port first...

  read(fd, buf, 100);


  if(SendCommand(fd, DeviceReadyCmd, 0) == 0)
    syslog(LOG_INFO, "dns320l-daemon startup complete, going to FanControl mode");
  else
  {
    syslog(LOG_ERR, "Error sending DeviceReady command, exit!\n");
    return EXIT_FAILURE;
  }

  while(1)
  {
    sleepCount = 0;
    temperature = SendCommand(fd, ThermalStatusGetCmd, 1);
    if(temperature > 0)
    {
      temperature = ThermalTable[temperature];
      syslog(LOG_DEBUG, "Read Temperature: %i\n", temperature);
      if(temperature < (tempLow - hysteresis))
      {
        if(fanSpeed != 0)
        {
          syslog(LOG_DEBUG, "Set Fan Stop\n");
          SendCommand(fd, FanStopCmd, 0);
          fanSpeed = 0;
        }
      }
      else if(temperature < tempLow)
      {
        if(fanSpeed > 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, 0);
          fanSpeed = 1;
        }
      }
      else if(temperature < (tempHigh - hysteresis))
      {
        if(fanSpeed != 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, 0);
          fanSpeed = 1;
        }
      }
      else if(temperature < tempHigh)
      {
        if(fanSpeed < 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, 0);
          fanSpeed = 1;
        }
      }
      else
      {
        if(fanSpeed != 2)
        {
          syslog(LOG_DEBUG, "Set Fan Full\n");
          SendCommand(fd, FanFullCmd, 0);
          fanSpeed = 2;
        }
      }
    }
    else
    {
      syslog(LOG_ERR, "Error reading Temperature!\n");
    }


    while((sleepCount  * pollTimeMs) < (fanPollTime * 1000))
    {
      if(((sleepCount * pollTimeMs) % (gpioPollTime* 1000)) == 0)
      {
        if(gpio_get_value(GPIO_BUTTON_POWER, &powerBtn, gpioDir) == 0)
        {
          if((powerBtn == 0) && !pressed)
          {
            pressed = 1;
            syslog(LOG_INFO, "Power Button Pressed, shutting down system!\n");
            SendCommand(fd, DeviceShutdownCmd, 0);
            execl("/sbin/shutdown", "shutdown", "-h", "now", (char *)0);
          }
        }

      }
      sleepCount++;

      ret=poll(fds,nfds,pollTimeMs); // Time out after pollTimeMs
      if (ret == -1){
        syslog(LOG_ERR, "poll");
        exit(EXIT_FAILURE);
      }
      for (i=0;(i<nfds) && (ret);i++)
      {
        if (!(fds+i)->revents)
          continue;
        ret--;
        if (((fds+i)->fd == ls) && ((fds+i)->revents & POLLIN))
        {
          /*
                     * Accept connection from socket ls:
                     * accepted connection will be on socket (fds+nfds)->fd.
                     */
          syslog(LOG_DEBUG, "POLLIN on ls. Accepting connection\n");
          namelength = sizeof (s_name);
          fds = (struct pollfd *)realloc(fds,(nfds+1)*sizeof(struct pollfd));
          (fds+nfds)->fd  = accept (ls, (struct sockaddr *)&s_name, &namelength);
          if ((fds+nfds)->fd == -1)
          {
            syslog(LOG_ERR, "accept");
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
          syslog(LOG_DEBUG, "POLLNVAL on socket. Freeing resource\n");
          nfds--;
          memcpy(fds+i,fds+i+1,nfds-i);
          fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
          continue;
        }
        if ((fds+i)->revents & POLLHUP)
        {
          syslog(LOG_DEBUG, "POLLHUP => peer reset connection ...\n");
          cleanup(0,(fds+i)->fd,2);
          nfds--;
          memcpy(fds+i,fds+i+1,nfds-i);
          fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
          continue;
        }
        if ((fds+i)->revents & POLLERR){
          syslog(LOG_DEBUG, "POLLERR => peer reset connection ...\n");
          cleanup(0,(fds+i)->fd,2);
          nfds--;
          memcpy(fds+i,fds+i+1,nfds-i);
          fds = (struct pollfd *)realloc(fds,nfds*sizeof(struct pollfd));
          continue;
        }
        if ((fds+i)->revents & POLLRDNORM)
        {
          retval = recv((fds+i)->fd, message, sizeof(message)-1, 0); // Don't forget the string terminator here!
          syslog(LOG_DEBUG, "-> (recv) retval = %d.\n",retval);  /* ped */
          msgIdx = retval;
          if (retval <=0)
          {
            if (retval == 0)
            {
              syslog(LOG_DEBUG, "recv()==0 => peer disconnected...\n");
              cleanup(1,(fds+i)->fd,2);
            }
            else
            {
              syslog(LOG_ERR, "receive");
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
            syslog(LOG_DEBUG, " \t -> (recv) retval = %d.\n", retval);
            if(retval > 0)
              msgIdx += retval - 2;
          }
          if(msgIdx > 1)
            if(message[msgIdx-1] == '\n')
              if(message[msgIdx-2] == '\r')
                message[msgIdx-2] = '\0';
              else
                message[msgIdx-1] = '\0';

          syslog(LOG_DEBUG, "Normal message :  %.*s\n",retval,message);
          msgIdx = HandleCommand(message, msgIdx, response, sizeof(response));
          retval = send((fds+i)->fd, response, strlen(response), 0);
          if((retval < 0) || (msgIdx == 1))
          {
            syslog(LOG_DEBUG, "send()==0 => peer disconnected...\n");
            cleanup(1,(fds+1)->fd, 2);
          }
          continue;
        }
      }
    }
  }
  iniparser_freedict(iniFile);
  return 0;
}
