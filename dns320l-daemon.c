/*

  Simple system daemon for D-Link DNS-320L

  (c) 2013 Andreas Boehler, andreas _AT_ aboehler.at

  This code is based on a few other people's work and in parts shamelessly copied.
  The ThermalTable was provided by Lorenzo Martignoni and the fan control 
  algorithm is based on his fan-daemon.py implementation.
  
  The MCU protocol was reverse engineered by strace() calls to up_send_daemon and
  up_read_daemon of the original firmware.



  This program is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program. If not, see <http://www.gnu.org/licenses/>.

*/

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
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include <syslog.h>
#include <iniparser.h>

#include "dns320l.h"


int ls;
int fd;

/** @file dns320l-daemon.c
    @brief Implementation of a free system daemon replacement for
           the D-Link DNS-320L NAS
    @author Andreas Boehler, andreas _AT_ aboehler.at
    @version 1.0
    @date 2013/09/12
*/

/** <i>Function</i> that reads a GPIO value from sysfs interface.
  @param gpio The GPIO number to read
  @param value Pointer where the value is to be put
  @param gpioDir Pointer containing the sysfs path to the GPIO subdir
  @return The GPIO's value
  */
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

/** <i>Function</i> that cleans up a socket after shutdown
  @param shut If 1, the socket is shutdown first
  @param s The socket to work on
  @param howmany Number of sockets to close?
*/
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

/** <i>Function</i> that is called by the OS upon sending a signal
    to the application
 @param sig The signal number received
*/
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

/** <i>Function</i> that sets interface attributes on a given
  serial port.
 @param fd The file descriptor (serial port) to work with
 @param speed The speed the interface to configure
 @param parity Use parity or not
 @return 0 on success, otherwise 1
*/
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

/** <i>Function</i> that sets an interface to either blocking
  or non-blocking mode
  @param fd The file descriptor to work with
  @param should_block Flag whether it should block or not
*/
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

/** <i>Function</i> that checks the first few bytes of the MCU's response
  whether it corresponds to the sent command
  @param buf The buffer to compare
  @param cmd The command that was sent
  @param len The lenght of the command
  @return SUCCESS on success, otherwise ERR_WRONG_ANSWER
*/
int CheckResponse(char *buf, char *cmd, int len)
{
  int i;
  int tmp;

  // Attention, 5 is hardcoded here and never checked!
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
  return SUCCESS;
}

/** <i>Function</i> that sends a command to the MCU and waits
  for response and/or ACK.
  @param fd The serial port to work on
  @param cmd The command to send
  @param outArray An array where the response shall be put, can be NULL for no response
  @return SUCCESS, ERR_WRONG_ANSWER or the number of bytes received
  */
int SendCommand(int fd, char *cmd, char *outArray)
{
  int n;
  int i;
  int j;

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
    // If outArray is not NULL, an answer was requested
    if(outArray != NULL)
    {
      CheckResponse(buf, cmd, i);
      // Copy the answer to the outArray
      for(j=0; j<i; j++)
      {
        outArray[j] = buf[j];
      }
      usleep(20000); // Give the µC some time to answer...

      // Wait for ACK from Serial
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
      syslog(LOG_DEBUG, "Returning %i read bytes\n", n);
      return j;
    }
    // Only wait for ACK if no response is expected
    else
    {
      return CheckResponse(buf, AckFromSerial, i);
    }
  }
}

/** <i>Function</i> that handles commands received by the socket
  and puts the response back into the retMessage buffer
  @param message The message that was received (the command)
  @param messageLen The lenght of the received message
  @param retMessage Pointer to an output array for the response message
  @parma bufSize The size of the message buffer
  @return 0 on success, 1 on failure, 2 for quit and 3 for daemon shutdown
*/
int HandleCommand(char *message, int messageLen, char *retMessage, int bufSize)
{
  int tmp;
  int len;
  int i;
  time_t rtcTime;
  time_t sysTime;
  struct timeval setTime;
  char timeStr[100];
  struct tm strTime;
  struct tm *strSetTime;
  char buf[15];
  char cmdBuf[15];
  
  syslog(LOG_DEBUG, "Handling Command: %s\n", message);

  // This is a very ugly list of if-else and strncmp calls...
  
  if(strncmp(message, "DeviceReady", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DeviceReady\n");
    if(SendCommand(fd, DeviceReadyCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "GetTemperature", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetTemperature\n");
    if(SendCommand(fd, ThermalStatusGetCmd, buf) > ERR_WRONG_ANSWER)
    {
      tmp = ThermalTable[(int)buf[5]];
      snprintf(retMessage, bufSize, "%d", tmp);
      len = strlen(retMessage);
      if(bufSize > 1)
      {
        retMessage[len] = '\n';
        retMessage[len+1] = '\0';
      }
    }
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
/*  else if(strncmp(message, "DeviceShutdown", strlen("DeviceShutdown")) == 0)
  {
    syslog(LOG_DEBUG, "DeviceShutdown");
    if(messageLen >= (strlen("DeviceShutdown") + 2))
    {
      //tmp = atoi(&message[strlen("DeviceShutdown") + 1]); // FIXME: The parameter is never passed, we default to 10s here..
      //printf("%s\n", tmp);
      if(SendCommand(fd, DeviceShutdownCmd, NULL) == SUCCESS)
        strncpy(retMessage, "OK\n", bufSize);
      else
        strncpy(retMessage, "ERR\n", bufSize);
    }
  }*/
  
  else if(strncmp(message, "quit", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "Quit\n");
    strncpy(retMessage, "Bye\n", bufSize);
    return 2;
  }
  
  else if(strncmp(message, "EnablePowerRecovery", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "EnablePowerRecovery");
    if(SendCommand(fd, APREnableCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "DisablePowerRecovery", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DisablePowerRecovery");
    if(SendCommand(fd, APRDisableCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "GetPowerRecoveryState", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetPowerRecoveryState");
    if(SendCommand(fd, APRStatusCmd, buf) > ERR_WRONG_ANSWER)
    {
      snprintf(retMessage, bufSize, "%d", buf[5]);
      len = strlen(retMessage);
      if(bufSize > 1)
      {
        retMessage[len] = '\n';
        retMessage[len+1] = '\0';
      }
    }
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "EnableWOL", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "EnableWOL");
    if(SendCommand(fd, WOLStatusEnableCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "DisableWOL", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "DisableWOL");
    if(SendCommand(fd, WOLStatusDisableCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "GetWOLState", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "GetWOLState");
    if(SendCommand(fd, WOLStatusGetCmd, buf) > ERR_WRONG_ANSWER)
    {
      snprintf(retMessage, bufSize, "%d", buf[5]);
      len = strlen(retMessage);
      if(bufSize > 1)
      {
        retMessage[len] = '\n';
        retMessage[len+1] = '\0';
      }
    }
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "PowerLedOn", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedOn");
    if(SendCommand(fd, PwrLedOnCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "PowerLedOff", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedOff");
    if(SendCommand(fd, PwrLedOffCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "PowerLedBlink", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "PowerLedBlink");
    if(SendCommand(fd, PwrLedBlinkCmd, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "systohc", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "systohc");
    // Copy the command to our buffer
    for(i=0;i<13;i++)
    {
      cmdBuf[i] = WDateAndTimeCmd[i];
    }
    sysTime = time(NULL);
    strSetTime = localtime(&sysTime);
    // Put the current local time into the command buffer
    cmdBuf[5] = (char)strSetTime->tm_sec;
    cmdBuf[6] = (char)strSetTime->tm_min;
    cmdBuf[7] = (char)strSetTime->tm_hour;
    cmdBuf[8] = (char)strSetTime->tm_wday;
    cmdBuf[9] = (char)strSetTime->tm_mday;
    cmdBuf[10] = (char)(strSetTime->tm_mon + 1);
    cmdBuf[11] = (char)(strSetTime->tm_year - 100);
    // And modify the values so that the MCU understands them...
    for(i=5;i<12;i++)
    {
      cmdBuf[i] = ((cmdBuf[i] / 10) << 4) + (cmdBuf[i] % 10);
    }
    if(SendCommand(fd, cmdBuf, NULL) == SUCCESS)
      strncpy(retMessage, "OK\n", bufSize);
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "hctosys", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "hctosys");
    // Retrieve RTC time first
    if(SendCommand(fd, RDateAndTimeCmd, buf) > ERR_WRONG_ANSWER)
    {
      for(i=5;i<12;i++)
      {
        buf[i] = (buf[i] & 0x0f) + 10 * ((buf[i] & 0xf0) >> 4); // The other end is a µC (doh!)
      }

      strTime.tm_year = (100 + (int)buf[11]);
      strTime.tm_mon = buf[10]-1;
      strTime.tm_mday = buf[9];
      strTime.tm_hour = buf[7];
      strTime.tm_min = buf[6];
      strTime.tm_sec = buf[5];
      strTime.tm_isdst = -1;
      rtcTime = mktime(&strTime);
      strcpy(timeStr, ctime(&rtcTime));
      // Retrieve system time
      sysTime = time(NULL);
      setTime.tv_sec = rtcTime;
      setTime.tv_usec = 0;
      // Set the time and print the difference on success
      if(settimeofday(&setTime, NULL) != 0)
        strncpy(retMessage, "ERR\n", bufSize);
      else
        snprintf(retMessage, bufSize, "RTC: %sSys: %sDiff: %.fs\n", timeStr, ctime(&sysTime), difftime(sysTime, rtcTime));
    }
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "ReadRtc", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "ReadRtc");
    if(SendCommand(fd, RDateAndTimeCmd, buf) > ERR_WRONG_ANSWER)
    {
      for(i=5;i<12;i++)
      {
        buf[i] = (buf[i] & 0x0f) + 10 * ((buf[i] & 0xf0) >> 4); // The other end is a µC (doh!)
      }
      strTime.tm_year = (100 + (int)buf[11]);
      strTime.tm_mon = buf[10]-1;
      strTime.tm_mday = buf[9];
      strTime.tm_hour = buf[7];
      strTime.tm_min = buf[6];
      strTime.tm_sec = buf[5];
      strTime.tm_isdst = -1;   
      rtcTime = mktime(&strTime);
      strcpy(timeStr, ctime(&rtcTime));         
      snprintf(retMessage, bufSize, "RTC: %s", timeStr);
    }
    else
    {
      strncpy(retMessage, "ERR\n", bufSize);
      return 1;
    }
  }
  
  else if(strncmp(message, "ShutdownDaemon", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "ShutdownDaemon");
    strncpy(retMessage, "OK\n", bufSize);
    return 3;
  }
  
  else if(strncmp(message, "help", messageLen) == 0)
  {
    syslog(LOG_DEBUG, "help");
    strncpy(retMessage, "Available Commands: DeviceReady, GetTemperature, "// DeviceShutdown, "
            "EnablePowerRecovery, DisablePowerRecovery, GetPowerRecoveryState, "
            "EnableWOL, DisableWOL, GetWOLState, PowerLedOn, "
            "PowerLedOff, PowerLedBlink, systohc, hctosys, ReadRtc, ShutdownDaemon, quit\n", bufSize);
  }
  else
  {
    strncpy(retMessage, "Command not Understood!\n", bufSize);
  }

  return 0;
}

/** <i>Main Function</i>
  @param argc The argument count
  @param argv The argument vector
  @return EXIT_SUCCESS on success, otherwise EXIT_ERROR
*/
int main(int argc, char *argv[])
{

  char *portname;
  char *gpioDir;
  int serverPort;
  int fanPollTime;
  int gpioPollTime;
  char response[500];
  int i;
  pid_t pid;
  pid_t sid;
  int powerBtn;
  int pressed;
  int opt;
  int sleepCount;
  int pollTimeMs;
  int goDaemon = 1;
  int debug = 0;
  int readRtcOnStartup = 0;
  char buf[100];
  char *configPath = "/etc/dns320l-daemon.ini";
  char msgBuf[15];
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

  // Parse command line arguments
  while((i = getopt(argc, argv, "fc:d")) != -1)
  {
    switch(i)
    {
      case 'f':
        goDaemon = 0;
        break;
      case 'd':
        debug = 1;
        goDaemon = 0;
        break;
      case 'c':
        configPath = optarg;
        break;
      case '?':
        if(optopt == 'c')
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
        else if (isprint (optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
          fprintf (stderr,
                   "Unknown option character `\\x%x'.\n",
                   optopt);
        fprintf(stderr, "Usage: %s [-f] [-c configPath] [-d]\n", argv[0]);
        fprintf(stderr, "       where\n");
        fprintf(stderr, "         -f              don't detach\n");
        fprintf(stderr, "         -c configPath   path to .ini\n");
        fprintf(stderr, "         -d              debug (implies -f)\n");
        return EXIT_FAILURE;
    }
  
  }
  
  // Register some signal handlers
  signal(SIGTERM, sighandler);
  signal(SIGINT, sighandler);
  
  // Load our configuration file or use default values 
  // if it doesn't exist!
  iniFile = iniparser_load(configPath);
  portname = iniparser_getstring(iniFile, "Serial:Port", "/dev/ttyS1");
  readRtcOnStartup = iniparser_getint(iniFile, "Daemon:SyncTimeOnStartup", 0);
  fanPollTime = iniparser_getint(iniFile, "Fan:PollTime", 15);
  tempLow = iniparser_getint(iniFile, "Fan:TempLow", 45);
  tempHigh = iniparser_getint(iniFile, "Fan:TempHigh", 50);
  hysteresis = iniparser_getint(iniFile, "Fan:Hysteresis", 2);
  gpioPollTime = iniparser_getint(iniFile, "GPIO:PollTime", 1);
  gpioDir = iniparser_getstring(iniFile, "GPIO:SysfsGpioDir", "/sys/class/gpio");
  serverPort = iniparser_getint(iniFile, "Daemon:ServerPort", 57367);

  // Setup syslog
  if(debug)
    setlogmask(LOG_UPTO(LOG_DEBUG));
  else
    setlogmask(LOG_UPTO(LOG_INFO));
  
  if(goDaemon)
    openlog("dns320l-daemon", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
  else
    openlog("dns320l-daemon", LOG_CONS | LOG_PID | LOG_NDELAY | LOG_PERROR, LOG_LOCAL1);
    
  if(goDaemon)
  {
    pid = fork();
    if(pid < 0)
    {
      syslog(LOG_ERR, "Forking failed.\n");
      return EXIT_FAILURE;
    }
    
    if(pid > 0)
    {
      return EXIT_SUCCESS;
    }
    // From here on we are the child process...
    umask(0);
    sid = setsid();
    if(sid < 0)
    {
      syslog(LOG_ERR, "Could not create process group\n");
      return EXIT_FAILURE;
    }
    
    if((chdir("/")) < 0)
    {
       syslog(LOG_ERR, "Could not chdir(\"/\")\n");
       return EXIT_FAILURE;
    }
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
  
  }




  // Open our socket server
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

  // Send the DeviceReady command to the MCU
  if(SendCommand(fd, DeviceReadyCmd, NULL) == SUCCESS)
    syslog(LOG_INFO, "dns320l-daemon startup complete, going to FanControl mode");
  else
  {
    syslog(LOG_ERR, "Error sending DeviceReady command, exit!\n");
    return EXIT_FAILURE;
  }
  
  if(readRtcOnStartup)
  {
    syslog(LOG_INFO, "Setting system clock from RTC...\n");
    if(HandleCommand("hctosys", 7, NULL, 0) != 0)
      syslog(LOG_ERR, "Error setting system time from RTC!\n");
  }

  // Go to endless loop and do the following:
  // Get the thermal status
  // Check temperature and adjust fan speeds
  // Wake every 1s to poll the power button GPIO
  // Wake every few ms to poll the sockets for connections
  // Sleep
  
  while(1)
  {
    sleepCount = 0;
    if(SendCommand(fd, ThermalStatusGetCmd, msgBuf) > ERR_WRONG_ANSWER)
      temperature = msgBuf[5];
    else
      temperature = 0;
    if(temperature > 0)
    {
      temperature = ThermalTable[temperature];
      syslog(LOG_DEBUG, "Read Temperature: %i\n", temperature);
      if(temperature < (tempLow - hysteresis))
      {
        if(fanSpeed != 0)
        {
          syslog(LOG_DEBUG, "Set Fan Stop\n");
          SendCommand(fd, FanStopCmd, NULL);
          fanSpeed = 0;
        }
      }
      else if(temperature < tempLow)
      {
        if(fanSpeed > 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, NULL);
          fanSpeed = 1;
        }
      }
      else if(temperature < (tempHigh - hysteresis))
      {
        if(fanSpeed != 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, NULL);
          fanSpeed = 1;
        }
      }
      else if(temperature < tempHigh)
      {
        if(fanSpeed < 1)
        {
          syslog(LOG_DEBUG, "Set Fan Half\n");
          SendCommand(fd, FanHalfCmd, NULL);
          fanSpeed = 1;
        }
      }
      else
      {
        if(fanSpeed != 2)
        {
          syslog(LOG_DEBUG, "Set Fan Full\n");
          SendCommand(fd, FanFullCmd, NULL);
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
            SendCommand(fd, DeviceShutdownCmd, NULL);
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
          if((retval < 0) || (msgIdx > 1))
          {
            syslog(LOG_DEBUG, "send()==0 => peer disconnected...\n");
            cleanup(1,(fds+1)->fd, 2);
          }
          if(msgIdx == 3)
          {
            syslog(LOG_INFO, "Shutting down dns320l-daemon...\n");
            return EXIT_SUCCESS;
          }
          continue;
        }
      }
    }
  }
  closelog();
  iniparser_freedict(iniFile);
  return EXIT_SUCCESS;
}
