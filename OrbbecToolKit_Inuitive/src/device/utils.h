#ifndef UTILS_H
#define UTILS_H

#include "qstring.h"
#include "qlist.h"

typedef struct Device_Resolution{
    int indexId;
    int width;
    int height;
    int fps;
    QString format;
}Device_Recolution_t;


typedef struct Uvc_Device{
    int cameraId;
    QString deviceName;
    QList<Device_Resolution> resolutions;
}Uvc_Device_t;


#endif // UTILS_H
