#ifndef KUNWEISENSOR_H
#define KUNWEISENSOR_H

#include "ftsensor.h"
#include <QSerialPort>
#include <QObject>
#include <QSerialPortInfo>
#include <QDebug>
#include <thread>
//#include <boost/bind.hpp>
#include <iostream>
#include <queue>
#include "serial.h"
#include <sys/timeb.h>

typedef enum{
    INIT = 0,
    CONTINUOUS_MODE,
    REQUEST_MODE,
    CLEAR,
}SENSOR_COMMAND;

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef int SOCKET_HANDLE;

typedef struct KunWeiResponseStruct {
    uint32 rdt_sequence;
    uint32 ft_sequence;
    uint32 status;
    float FTData[6];
} KunWeiResponse;


class KunWeiSensor: public QObject, public FTSensor
{
public:
    KunWeiSensor();
    ~KunWeiSensor();


    int writeCommand(SENSOR_COMMAND command);
    int readSensor(unsigned char m_rec_data[]);

public:
    bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]);
    bool initialFTSensor();
    bool uninitialFTSensor();
    void getFTSensorRange(double range[]);

//private slots:
    void handleSensorData();
    void handleSensorData2();

private:
    std::string port_name_;
    const float gravity_acc_ = 9.81f;
    Serial *serial_port_;
    KunWeiResponse ftResponse;
    bool read_success_;
    static double s_range[SENSOR_DIMENSION];
    std::thread* read_sensor_data_;
    bool sensorRunning;

    int m_serial_fd;
    char command_head_;
};
#endif // KUNWEISENSOR_H
