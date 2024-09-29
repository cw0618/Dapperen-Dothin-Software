/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/

#ifndef PNGPARSER_H
#define PNGPARSER_H

#include <QThread>
#include <3rd/obpng/include/orbbec_png_info.h>


class PngParser : public QThread
{
    Q_OBJECT
private:

    QString filePath;

    PNG_FILE png_file;
    bool png_file_ok;
public:
    PngParser(QObject *parent = 0);

    void parsePng(QString filePath);


    PNG_FILE* getPng_file(){
        return &png_file;
    }

    bool isPng_info_ok() const{
        return png_file_ok;
    }

protected:
    // QThread interface
    void run();
signals:
    void pngParseFinish(int error);
};

#endif // PNGPARSER_H
