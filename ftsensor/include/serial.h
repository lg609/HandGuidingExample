#ifndef KUNWEI_SERIAL_H
#define KUNWEI_SERIAL_H
#include  <stdio.h>
#include  <stdlib.h>
#include  <unistd.h>
#include  <sys/types.h>
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <string.h>

class Serial
{
    public:
    Serial(){}
    ~Serial(){}

    int openPort(const char* device_name);
    int SetPara(int serialfd,int speed=8,int databits=8,int stopbits=1,int parity=0);
    int WriteData(int fd,const char *data,int datalength);
    int ReadData(int fd,unsigned char *data,int datalength);
    void closePort(int fd);
    int BaudRate( int baudrate);
};
#endif
