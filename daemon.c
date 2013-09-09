#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <memory.h>
#include "dns320l.h"

#define FAN_POLL_TIME    15

int
set_interface_attribs (int fd, int speed, int parity)
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

void
set_blocking (int fd, int should_block)
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
            usleep(10000); // Give the µC some time to answer...

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

int main(int args, char *argv[])
{

    char *portname = "/dev/ttyS1"; // We hardcode the path, as this daemon is inteded to run on one box only
    int fd;
    int n;
    int i;
    int sleepCount;
    int sleepTimeUs;
    char buf [100];
    int temperature;
    int fanSpeed;
    
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

    
    sleepCount = 0;
    sleepTimeUs = 1000 * 100; // Sleep 100ms for every loop
    fanSpeed = 0;
    while(1)
    {
        sleepCount = 0;
        temperature = SendCommand(fd, FanStatusGetCmd, 1);
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
        
        
        while((sleepCount  * sleepTimeUs) < (FAN_POLL_TIME * 1000 * 1000))
        {
            usleep(sleepTimeUs); // Sleep 100ms, replace by socket polling code for IPC
            sleepCount++;
        }
    
    }
    
    return 0;
}
