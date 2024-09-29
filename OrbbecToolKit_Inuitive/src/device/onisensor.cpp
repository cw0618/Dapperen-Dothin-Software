#include "onisensor.h"

OniSensor::OniSensor(const string & name) : sensor_name_(name)
{
    if (0 == name.compare("mx6000"))
    {
        device_ = shared_ptr<SensorBase>(mx6000::Instance());
    }
};
