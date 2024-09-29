#ifndef OBKIT_COMMON_H
#define OBKIT_COMMON_H

#include <QString>
#include <QStringList>
#pragma execution_character_set("utf-8")
#define OBKIT_VERSION "1.0.3.37"

#define R_TYPE_FILED_PHASE   0xF0
#define R_TYPE_FILED_AMP     0xE0
#define R_TYPE_FILED_DEPTH   0xD0
#define R_TYPE_FILED_PCLOUD  0xC0
#define R_TYPE_FILED_IR      0xB0
#define PCLOUD_MOVE_TO_CENTER 0

#define POINT_CLOUD_MAP 100
#define IR_MAP 101
#define MIPI_RAW 102

#define IS_EXTERNAL_VERSION 0


typedef QString (*cmdExec)(int argc, QStringList argv, void* handle);
typedef struct {
    cmdExec cmd_executer;
    void* handle;
} Cmder;


struct BufBean{
    int w;
    int h;
    int size;
    char *data;
    bool valid;
    int pointSize;
};

struct PCloudBean{
    int    w;
    int    h;
    float  *data;
    int    pointCount;
    int    pointSize;
    bool   valid;
} ;

static const char* OutputModel[] = {
    "A-B",
    "A+B",
    "A",
    "B",
    "A&B",
};

static const char* OutputModel_33D[] = {
    "4TAP_DUAL_FREQ_SHUFFLE",
};
typedef struct _CaptureBean{
    char* buf;
    int size;
    int w;
    int h;
    int index;

    char extInfo[40];
    int64_t timeStamp;
}CaptureBean;

typedef struct RGB888{
    uint8_t R;
    uint8_t G;
    uint8_t B;
}RGB888;

class Common {

};
#endif // OBKIT_COMMON_H
