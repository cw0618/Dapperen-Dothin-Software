#include "pngparser.h"
#include <QDebug>
/** \class CaptureDialog
*
* png图片信息解释类
*
*/

PngParser::PngParser(QObject *parent):QThread(parent)
{
    memset(&png_file, 0, sizeof(PNG_FILE));
    filePath = "";
    png_file_ok = false;

}

void PngParser::parsePng(QString filePath){
    this->filePath = filePath;
    start();
}

void PngParser::run(){
    png_file_ok = false;


    if(filePath.isEmpty()){
        emit pngParseFinish(-1);
        return;
    }

    QByteArray ba = filePath.toLocal8Bit();
    char* file_c = ba.data();

    FILE* fin = fopen(file_c, "rb");
    if(fin == NULL){
        emit pngParseFinish(-2);
        return;
    }

    int ret = 0;
    //read png chunk info
    ret = load_png_file(fin, &png_file);
    if(ret == 0){
        png_file_ok = true;
    }else{
        qDebug() << "PngParser run: load_png_file failed:";
        ret = -3;
    }


    if(fin != NULL){
        fclose(fin);
    }

    emit pngParseFinish(ret);
}




