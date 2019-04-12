#include "../include/KunWeiSensor.h"

double KunWeiSensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};

KunWeiSensor::KunWeiSensor()
{
}

bool KunWeiSensor::initialFTSensor()
{
    bool ret = false;

    serial_port_ = new QSerialPort(port_name_);

    if(serial_port_->open(QIODevice::ReadWrite))
    {
        serial_port_->setBaudRate(QSerialPort::Baud512000);
        serial_port_->setDataBits(QSerialPort::Data8);
        serial_port_->setParity(QSerialPort::NoParity);
        serial_port_->setStopBits(QSerialPort::OneStop);
        serial_port_->setFlowControl(QSerialPort::NoFlowControl);
        //        QObject::connect(serial_port_, &QSerialPort::readyRead, this, &KunWeiSensor::handleSensorData);
        read_sensor_data_ = new std::thread(boost::bind(&KunWeiSensor::handleSensorData, this));
        //        read_sensor_data_2 = new std::thread(boost::bind(&KunWeiSensor::handleSensorData2, this));
        qDebug()<<"Start the read sensor thread!";
        ret = true;
    }

    return ret;
}

bool KunWeiSensor::uninitialFTSensor()
{
    serial_port_->close();
    if(serial_port_ != NULL)
        delete serial_port_;
    return true;
}


void KunWeiSensor::getFTSensorRange(double range[])
{
    memcpy(range, s_range, sizeof(double)*SENSOR_DIMENSION);
}

bool KunWeiSensor::obtainFTSensorData(float m_ftData[SENSOR_DIMENSION])
{
    //    if(!read_success_)
    //        return false;
    memcpy(m_ftData, ftResponse.FTData, sizeof(float)*SENSOR_DIMENSION);
    return true;
}

static long ncount = 0;
static long mcount = 0;
static std::vector<unsigned char> dataVector;
void KunWeiSensor::handleSensorData()
{
    while(true)
    {
        while(serial_port_->waitForReadyRead(2))
        {
            ncount++;
            QByteArray sensor_data_ = serial_port_->readAll();
            serial_port_->clear();
            //        for(int i = 0; i < sensor_data_.size(); i++)
            //            dataVector.push_back((unsigned char)sensor_data_[i]);

//                    std::cout<<"rate:::::::"<<mcount*100.0/ncount<<std::endl;
            if(sensor_data_.size() >= 27)
                if(sensor_data_[0] != 0x48 || sensor_data_[25] != 0x0D || sensor_data_[26] != 0x0A)
                    read_success_ = false;
                else
                {
                    mcount++;
                    unsigned char c[4] ;
                    for(int i = 0; i < SENSOR_DIMENSION; i++)
                    {
                        c[0] = sensor_data_[1+4*i];
                        c[1] = sensor_data_[2+4*i];
                        c[2] = sensor_data_[3+4*i];
                        c[3] = sensor_data_[4+4*i];
                        memcpy(&ftResponse.FTData[i], c, sizeof(float));
                        //                     memcpy(&ftResponse.FTData[i], sensor_data_+1+4*i, sizeof(float));
                        ftResponse.FTData[i] = ftResponse.FTData[i] * gravity_acc_;
                    }
                    read_success_ = true;
                }
            //        usleep(1000);
        }
    }
}

void KunWeiSensor::handleSensorData2()
{
    while(true)
    {
        unsigned char data;
        while(dataVector.size() >= 27)
        {
            data = dataVector.front();
            if(data == 0x48)
            {
                if(dataVector[25] == 0x0D && dataVector[26] == 0x0A)
                {
                    unsigned char c[4];
                    for(int i = 0; i < SENSOR_DIMENSION; i++)
                    {
                        c[0] = dataVector[1+4*i];
                        c[1] = dataVector[2+4*i];
                        c[2] = dataVector[3+4*i];
                        c[3] = dataVector[4+4*i];
                        memcpy(&ftResponse.FTData[i], c, sizeof(float));
                        //                     memcpy(&ftResponse.FTData[i], sensor_data_+1+4*i, sizeof(float));
                        ftResponse.FTData[i] = ftResponse.FTData[i] * gravity_acc_;
                    }
                    for(int i = 0;i<26;i++)
                    {
                        std::vector<unsigned char>::iterator k = dataVector.begin();
                        dataVector.erase(k);
                    }
                }
                else
                {
                    std::vector<unsigned char>::iterator k = dataVector.begin();
                    dataVector.erase(k);
                }
            }
            else
            {
                std::vector<unsigned char>::iterator k = dataVector.begin();
                dataVector.erase(k);
            }
            //            usleep(100);
        }
    }
}

