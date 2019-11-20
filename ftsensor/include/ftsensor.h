#ifndef FTSENSOR_H2
#define FTSENSOR_H2

#define SENSOR_DIMENSION 6
using namespace std;

enum SENSOR_TYPE
{
    OPTOFORCE = 0,
    ROBOTIQ,
    ATI,
    KunWei
};

typedef struct
{
    int trackEnable;               //T
    int paraChanged;  //RT
    double sensorData[6]; //T
}FTSensorData;

class FTSensor
{
public:
    FTSensor();
    ~FTSensor();

    virtual bool obtainFTSensorData(float m_ftData[SENSOR_DIMENSION]) = 0;
    virtual bool initialFTSensor() = 0;
    virtual void getFTSensorRange(double range[]) = 0;
    virtual bool uninitialFTSensor() = 0;

};

#endif // FTSENSOR_H2
