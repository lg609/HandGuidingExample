#include "../include/ATISensor.h"

#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<netdb.h>
#include<sys/types.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<errno.h>

int socketFd;
struct  sockaddr_in serverAddr;

double num1, num2, num3, num4, num5, num6;


double ATISensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};

 ATISensor::ATISensor(std::string IPAddress)
 {
    ipAddress = /*IPAddress*/"192.168.1.1";
    memset(&ftResponse, 0, sizeof(ATIResponse));
 }

// ATISensor::~ATISensor()
// {
//     sensorRunning = false;
//     usleep(3*1000);
//     close(socketFd);
//     if(read_sensor_data_->joinable())
//         read_sensor_data_->join();
// }

 void ATISensor::getFTSensorRange(double range[])
 {
     memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
 }

/* Sleep ms milliseconds */
void ATISensor::mySleep(unsigned long ms)
{
    usleep(ms * 1000);
}

int ATISensor::connectSensor(SOCKET_HANDLE * handle, uint16 port)
{
    struct sockaddr_in addr;
    struct hostent *he;
    int err;

    *handle = socket(AF_INET, SOCK_DGRAM, /*IPPROTO_TCP*/0);

    if (*handle == -1)
    {
        fprintf(stderr, "Socket could not be opened.\n");
        return -2;
    }
    he = gethostbyname(ipAddress.c_str());
    if(he == 0)
        return -4;
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    fprintf(stdout, "Connecting to ATI\r\n");
    fflush(stdout);
    err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0) {
        return -3;
    }
    return 0;
}

void ATISensor::closeSensor(SOCKET_HANDLE * handle)
{
    close(*handle);
}



int ATISensor::readSensor(SOCKET_HANDLE * socket, ATIResponse &resp)
{
    byte request[8];			/* The request data sent to the Net F/T. */
    byte response[36];			/* The raw response data received from the Net F/T. */
    int readSuccess;
    int sendSuccess;
    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(ATI_COMMAND); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
    try
    {
        sendSuccess = send(*socket, &request, 8, 0);
    }
    catch(...)
    {
        return 3;

    }
    if (sendSuccess < 0) {
        return sendSuccess;
    }
    readSuccess = recv(*socket, response, 36, 0);
    resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
    resp.ft_sequence = ntohl(*(uint32*)&response[4]);
    resp.status = ntohl(*(uint32*)&response[8]);
    for(int i = 0; i < 6; i++)
    {
        resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
    }

    if (readSuccess != 36) {
        return 1;
    }

    return 0;
}


void ATISensor::showResponse(ATIResponse * r, float ft[])
{

    double Fx = (double)r->FTData[0] / 1000000;
    double Fy = (double)r->FTData[1] / 1000000;
    double Fz = (double)r->FTData[2] / 1000000;
    double Tx = (double)r->FTData[3] / 1000000;
    double Ty = (double)r->FTData[4] / 1000000;
    double Tz = (double)r->FTData[5] / 1000000;
    ft[0] = Fx; ft[1] = Fy; ft[2] = Fz; ft[3] = Tx; ft[4] = Ty; ft[5] = Tz;
}


bool ATISensor::initialFTSensor()
{
    socketFd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serverAddr,0,sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    inet_pton (AF_INET, "127.0.0.1", &serverAddr.sin_addr);
//    inet_pton (AF_INET, "192.168.1.80", &serverAddr.sin_addr);
    serverAddr.sin_port = htons(8877);

    struct timeval timeout;
    timeout.tv_sec  = 2;
    timeout.tv_usec = 0;
    setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    timeout.tv_sec  = 3;
    timeout.tv_usec = 0;
    setsockopt(socketFd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if( connect(socketFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == 0 )
    {
        printf("connect server success\n");
        sensorRunning = true;
        read_sensor_data_ = new std::thread(boost::bind(&ATISensor::handleSensorData, this));
        return true;
    }
    else
        return false;
//    read_sensor_data_ = new std::thread(boost::bind(&ATISensor::handleSensorData, this));


    /*
    if (connectSensor(&socketHandle, ATI_PORT) != 0)
    {
        fprintf(stderr, "Could not connect to device...\r\n");
        return false;
    }
    else
    {
        fprintf(stdout, "Connected to Ethernet DAQ\r\n");
        fflush(stdout);
        return true;
    }
    */

}

void ATISensor::handleSensorData()
{
    char recvBuff[1024] = {0};
    while(sensorRunning)
    {
        struct timeb tb;
        ftime(&tb);
        int t1 = tb.millitm;

        memset(recvBuff, sizeof(recvBuff), 0);

        int recvResult = recv(socketFd, recvBuff, 1024, 0);
//        printf("%d\n", t1);

        if(recvResult>0)
        {
//            printf("%s\n",&recvBuff[0]);

            char *ret = NULL;

            num1 = atof(recvBuff);
//            printf("%f\n", num1);

            ret  = strchr(recvBuff, '#')+1;
            num2 = atof(ret);
//            printf("%f\n", num2);

            ret  = strchr(ret, '#')+1;
            num3 = atof(ret);
//            printf("%f\n", num3);

            ret  = strchr(ret, '#')+1;
            num4 = atof(ret);
//            printf("%f\n", num4);

            ret  = strchr(ret, '#')+1;
            num5 = atof(ret);
//            printf("%f\n", num5);

            ret  = strchr(ret, '#')+1;
            num6 = atof(ret);
//            printf("%f\n", num6);
        }
        ftime(&tb);
        int t2 = tb.millitm;
        int sleepT = t2 - t1;
        if(sleepT < 0) sleepT += 1000;
        double ts = (4-sleepT);
        if(ts < 0)
        {
            ts = 0;
//            std::cout<<"FTData: err"<<std::endl;
        }
        usleep(ts*1000);
    }
}

bool ATISensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
    /*int readSuccess;
    try
    {
        readSuccess = readSensor(&socketHandle, ftResponse);
    }
    catch(...)
    {

    }

    if (readSuccess == 0)
    {
        showResponse(&ftResponse, m_ftData);
    }
    else
    {
        fprintf(stderr, "Could not read F/T data, error code: %d\r\n", readSuccess);
    }*/
    m_ftData[0] = num1 / 1000000;
    m_ftData[1] = num2 / 1000000;
    m_ftData[2] = num3 / 1000000;
    m_ftData[3] = num4 / 1000000;
    m_ftData[4] = num5 / 1000000;
    m_ftData[5] = num6 / 1000000;

    return true;
}

bool ATISensor::uninitialFTSensor()
{
    close(socketHandle);
    return true;
}

