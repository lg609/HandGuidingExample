#include "../include/KunWeiSensor.h"

double KunWeiSensor::s_range[SENSOR_DIMENSION] = {300,300,800,10,10,15};
//char INIT[4] ={ 0x43,0xAA, 0x0D, 0x0A};
//char CONTINUOUS_MODE[4]= { 0x48,0xAA,0x0D, 0x0A};
//char REQUEST_MODE[4] ={ 0x49,0xAA,0x0D,0x0A};
//char CLEAR[4]= {0x47, 0xAA,0x0D, 0x0A};
#define REC_DATA_LEN  60


KunWeiSensor::KunWeiSensor()
{
    port_name_ = "/dev/ttyUSB0";
    memset(&ftResponse, 0, sizeof(KunWeiResponse));
    //    std::cout<<"port_name:"<<port_name_<<std::endl;
    serial_port_ = new Serial();
    m_serial_fd = serial_port_->openPort(port_name_.c_str());
}

KunWeiSensor::~KunWeiSensor()
{
    if(read_sensor_data_->joinable())
        read_sensor_data_->join();
    usleep(5*1000);
    if(read_sensor_data_ != NULL)
        delete read_sensor_data_;
    serial_port_->closePort(m_serial_fd);
    if(serial_port_ != NULL)
        delete serial_port_;

}
void fun()
{}

bool KunWeiSensor::initialFTSensor()
{
    bool ret = false;

//    serial_port_ = new QSerialPort(port_name_);

//    if(serial_port_->open(QIODevice::ReadWrite))
//    {
//        serial_port_->setBaudRate(/*QSerialPort::Baud9600*/230400);
//        serial_port_->setDataBits(QSerialPort::Data8);
//        serial_port_->setParity(QSerialPort::NoParity);
//        serial_port_->setStopBits(QSerialPort::OneStop);
//        serial_port_->setFlowControl(QSerialPort::NoFlowControl);
//        sensorRunning = true;
//        //        QObject::connect(serial_port_, &QSerialPort::readyRead, this, &KunWeiSensor::handleSensorData);
//        read_sensor_data_ = new std::thread(boost::bind(&KunWeiSensor::handleSensorData2, this));
//        qDebug()<<"Start the read sensor thread!";
//        ret = true;
//    }
    if(m_serial_fd > 0)
    {
        serial_port_->SetPara(m_serial_fd,7);
        usleep(5*1000);
        writeCommand(INIT);
        writeCommand(CLEAR);
        writeCommand(INIT);
//        QSerialPort qq;
//        qq.setFlowControl(QSerialPort::NoFlowControl);
        sensorRunning = true;
        read_sensor_data_ = new std::thread(std::bind(&KunWeiSensor::handleSensorData2, this));
        qDebug()<<"Start the read sensor thread!";
        ret = true;
    }
    else
    {
        printf("Open serial port failed!\n");
        ret = false;
    }


    return ret;
}

bool KunWeiSensor::uninitialFTSensor()
{
    sensorRunning = false;
    if(read_sensor_data_->joinable())
        read_sensor_data_->join();
    usleep(3*1000);
    serial_port_->closePort(m_serial_fd);
    if(read_sensor_data_ != NULL)
        delete read_sensor_data_;
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
    memcpy(m_ftData, ftResponse.FTData, sizeof(float)*SENSOR_DIMENSION);
    return true;
}

int KunWeiSensor::writeCommand(SENSOR_COMMAND command)
{
    char data[4]= {0};
    switch(command)
    {
        case INIT:data[0] = 0x43; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case CONTINUOUS_MODE:data[0] = 0x48; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case REQUEST_MODE:data[0] = 0x49; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
        case CLEAR:data[0] = 0x47; data[1] = 0xAA; data[2] = 0x0D; data[3] = 0x0A;break;
    }

    command_head_ = data[0];
    if(serial_port_->WriteData(m_serial_fd,data,strlen(data)) < 0)
    {
           printf("Write Data Fail!\n");
           return -1;
    }
    return 0;
}

int KunWeiSensor::readSensor(unsigned char m_rec_data[REC_DATA_LEN])
{
    int m_rec_data_length = serial_port_->ReadData(m_serial_fd,m_rec_data,REC_DATA_LEN);
    if( m_rec_data_length > 0 )
      return m_rec_data_length;
    else
      return -1;
}

static long ncount = 0;
static long mcount = 0;
static std::vector<unsigned char> dataVector;
void KunWeiSensor::handleSensorData()
{
    QByteArray sensor_data_;
    while(sensorRunning)
    {
        while(sensorRunning /*&& serial_port_->waitForReadyRead(2)*/)
        {
//            ncount++;
//            QByteArray sensor_data_ = serial_port_->readAll();
//            serial_port_->clear();

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
        }
    }
}


void KunWeiSensor::handleSensorData2()
{
    unsigned char m_rec_data[REC_DATA_LEN];
    int control_period_ = 3;
    int data_size;
    while(sensorRunning)
    {

            struct timeb tb;
            ftime(&tb);
            int t1 = tb.millitm;
            setbuf(stdin,NULL);//设置缓冲区
            writeCommand(REQUEST_MODE);
            int data_size = readSensor(m_rec_data);
            int index = 0;

            if(data_size >= 12)
            {
                while(m_rec_data[index] != command_head_)
                    index++;
                unsigned int temp_data = 0;
                if(m_rec_data[index +10] == 0x0d && m_rec_data[index +11] == 0x0a)
                {
                    if ((m_rec_data[index +1] & 0x80) > 0)
                    {
                        temp_data = m_rec_data[index +1];
                        temp_data = temp_data|0xFFFFFF00;
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +2] >> 4);
                    }
                    else
                    {
                        temp_data = m_rec_data[index +1];
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +2] >> 4);
                    }
                    ftResponse.FTData[0] = (int)temp_data * (float)0.009765625 * gravity_acc_;
                    temp_data = 0;
                    if ((m_rec_data[index +2] & 0x08) > 0)
                    {
                        temp_data = m_rec_data[index +2];
                        temp_data = temp_data|0xFFFFFFF0;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +3];
                    }
                    else
                    {
                        temp_data = m_rec_data[index + 2];
                        temp_data = temp_data & 0x0F;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +3];
                    }
                    ftResponse.FTData[1] = (int)temp_data * (float)0.009765625 * gravity_acc_;
                    temp_data = 0;
                    if ((m_rec_data[index +4] & 0x80) > 0)
                    {
                        temp_data = m_rec_data[index +4];
                        temp_data = temp_data|0xFFFFFF00;
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +5] >> 4);
                    }
                    else
                    {
                        temp_data = m_rec_data[index +4];
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +5] >> 4);
                    }
                    ftResponse.FTData[2] = (int)temp_data * (float)0.009765625 * gravity_acc_;
                    temp_data = 0;
                    if ((m_rec_data[index +5] & 0x08) > 0)
                    {
                        temp_data = m_rec_data[index +5];
                        temp_data = temp_data|0xFFFFFFF0;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +6];
                    }
                    else
                    {
                        temp_data = m_rec_data[index +5];
                        temp_data = temp_data & 0x0F;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +6];
                    }
                    ftResponse.FTData[3] = (int)temp_data * (float)0.000390625 * gravity_acc_;
                    temp_data = 0;
                    if ((m_rec_data[index +7] & 0x80) > 0)
                    {
                        temp_data = m_rec_data[index +7];
                        temp_data = temp_data|0xFFFFFF00;
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +8] >> 4);
                    }
                    else
                    {
                        temp_data = m_rec_data[index +7];
                        temp_data = temp_data << 4;
                        temp_data = temp_data | (uint)(m_rec_data[index +8] >> 4);
                    }
                    ftResponse.FTData[4] = (int)temp_data * (float)0.000390625 * gravity_acc_;
                    temp_data = 0;
                    if ((m_rec_data[index +8] & 0x08) > 0)
                    {
                        temp_data = m_rec_data[index +8];
                        temp_data = temp_data|0xFFFFFFF0;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +9];
                    }
                    else
                    {
                        temp_data = m_rec_data[index +8];
                        temp_data = temp_data & 0x0F;
                        temp_data = temp_data << 8;
                        temp_data = temp_data | m_rec_data[index +9];
                    }
                    ftResponse.FTData[5] = (int)temp_data * (float)0.000390625 * gravity_acc_;
                }
            }

            ftime(&tb);
            int t2 = tb.millitm;
            int sleepT = t2 - t1;
            if(sleepT < 0) sleepT += 1000;
            double ts = (control_period_-sleepT);
            if(ts < 0)
            {
                ts = 0;
    //            std::cout<<"FTData: err"<<std::endl;
            }
            usleep(ts*1000);
    }
}
