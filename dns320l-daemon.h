#ifndef DNS320L_DAEMON_H
#define DNS320L_DAEMON_H


typedef struct
{
  int syncOnStartup;
  int syncOnShutdown;
  int fanPollTime;
  int pollGpio;
  int gpioPollTime;
  int serverPort;
  int goDaemon;
  int debug;
  char *gpioDir;
  char *portName;
  int tempLow;
  int tempHigh;
  int hysteresis;

} DaemonConfig;







#endif //DNS320L_DAEMON_H
