#-------------------------------------------------
#
# Project created by QtCreator 2019-11-22T14:54:59
#
#-------------------------------------------------
DESTDIR             = Bin
MOC_DIR             = $$PWD/buildobj
win32:OBJECTS_DIR   = $$PWD/buildobj
UI_DIR              = $$PWD/GeneratedFiles
CONFIG += c++11
QT += core gui
QT += opengl
QT += core xml
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OrbbecSensor
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
    src/main.cpp \
    src/render/obglwindow.cpp \
    src/calculate/calc.cpp \
    src/device/onisensor.cpp \
    src/device/OniStream.cpp \
    src/device/sensor_base.cpp \
    src/device/SubStream.cpp \
    src/device/mx6000.cpp \
    src/config/iniconfig.cpp \
    src/device/tuningwindow.cpp \
    src/weight/CustomWindow.cpp \
    src/mainwindowstyle.cpp \
    src/weight/videowindow.cpp \
    src/dialog/capturedialog.cpp \
    src/dialog/aboutdialog.cpp \
    src/dialog/moredialog.cpp \
    src/cameracontrolview.cpp \
    src/orbbec/ObPng.cpp \
    src/device/capturethread.cpp \
    src/ircontrolview.cpp \
    src/dialog/messagetip.cpp \
    src/commoncontrolview.cpp \
    src/depthcontrolview.cpp \
    src/config/minIni.cpp \
    src/phasecontrolview.cpp \
    src/device/obstream.cpp \
    src/device/OniStream.cpp \
    src/device/opendevicethread.cpp \
    src/weight/commoncontrolwidget.cpp \
    src/weight/devicestatuswidget.cpp \
    src/csv/CSVFile.cpp \
    src/weight/commoncontrolview.cpp \
    src/weight/switchbutton.cpp \
    src/weight/stylecombobox.cpp \
    src/opengl/mainGLWidget.cpp \
    src/opengl/renderPipeLine.cpp \
    src/opengl/renderShader.cpp \
    src/opengl/rendertextpipeline.cpp \
    src/opengl/shape.cpp \
    src/opengl/textureBuffer.cpp \
    src/device/imageutils.cpp \
    src/csv/csvbean.cpp \
    src/config/xmlconfig.cpp \
    src/config/xmlstreambean.cpp \
    src/dialog/streamtestdialog.cpp \
    src/weight/streamtestthread.cpp \
    src/device/wmffunction.cpp \
    src/weight/uvccontrolwidget.cpp \
    src/weight/uvccolorwidget.cpp \
    src/calculate/skeletonindex.cpp \
    src/render/circlerender.cpp \
    src/device/aibody.cpp \
    src/weight/aistreamcontrol.cpp \
    src/device/pngparser.cpp \
    src/pnginfowindow.cpp \
    src/cmd/commandmanager.cpp \
    src/cmd/mplaintextedit.cpp \
    src/cmd/widgetcmd.cpp

HEADERS += \
    src/render/obglwindow.h \
    src/calculate/calc.h \
    src/device/onisensor.h \
    src/device/sensor_base.h \
    src/device/SubStream.h \
    src/device/mx6000.h \
    src/config/iniconfig.h \
    src/device/tuningwindow.h \
    src/weight/CustomWindow.h \
    src/mainwindowstyle.h \
    src/weight/videowindow.h \
    src/dialog/capturedialog.h \
    src/dialog/aboutdialog.h \
    src/dialog/moredialog.h \
    src/cameracontrolview.h \
    src/orbbec/ObPng.h \
    src/stb_image.h \
    src/stb_image_write.h \
    src/device/capturethread.h \
    src/ircontrolview.h \
    src/dialog/messagetip.h \
    src/commoncontrolview.h \
    src/depthcontrolview.h \
    src/config/minIni.h \
    src/device/typedef.h \
    src/device/obutils.h \
    src/device/tofinfo.h \
    src/phasecontrolview.h \
    src/device/obstream.h \
    src/device/OniStream.h \
    src/device/opendevicethread.h \
    src/weight/commoncontrolwidget.h \
    src/weight/devicestatuswidget.h \
    src/csv/csvfile.h \
    src/weight/commoncontrolview.h \
    src/weight/switchbutton.h \
    src/weight/stylecombobox.h \
    src/render/glcolorwindow.h \
    src/opengl/mainGLWidget.h \
    src/opengl/renderPipeLine.h \
    src/opengl/renderShader.h \
    src/opengl/rendertextpipeline.h \
    src/opengl/shape.h \
    src/opengl/textureBuffer.h \
    src/device/imageutils.h \
    src/csv/csvbean.h \
    src/config/xmlconfig.h \
    src/config/xmlstreambean.h \
    src/dialog/streamtestdialog.h \
    src/weight/streamtestthread.h \
    src/device/wmffunction.h \
    src/device/utils.h \
    src/weight/uvccontrolwidget.h \
    src/weight/uvccolorwidget.h \
    src/calculate/skeletonindex.h \
    src/render/circlerender.h \
    src/device/aibody.h \
    src/weight/aistreamcontrol.h \
    src/device/pngparser.h \
    src/pnginfowindow.h \
    src/cmd/commandmanager.h \
    src/cmd/mplaintextedit.h \
    src/cmd/widgetcmd.h

FORMS += \
    src/device/tuningwindow.ui \
    src/weight/customwindow.ui \
    src/mainwindowstyle.ui \
    src/weight/videowindow.ui \
    src/dialog/capturedialog.ui \
    src/dialog/aboutdialog.ui \
    src/dialog/moredialog.ui \
    src/cameracontrolview.ui \
    src/ircontrolview.ui \
    src/dialog/messagetip.ui \
    src/commoncontrolview.ui \
    src/depthcontrolview.ui \
    src/phasecontrolview.ui \
    src/weight/commoncontrolwidget.ui \
    src/weight/devicestatuswidget.ui \
    src/weight/commoncontrolview.ui \
    src/weight/stylecombobox.ui \
    src/dialog/streamtestdialog.ui \
    src/weight/uvccontrolwidget.ui \
    src/weight/uvccolorwidget.ui \
    src/weight/aistreamcontrol.ui \
    src/pnginfowindow.ui \
    src/cmd/widgetcmd.ui
INCLUDEPATH += $$PWD/3rd/openni2/Include
DEPENDPATH += $$PWD/3rd/openni2/x86-Release
# rc
RC_ICONS = $$PWD/res/image/Orbbec.ico
INCLUDEPATH += $$PWD/3rd

win32: LIBS += -L$$PWD/3rd/freetype/win32/ -lfreetype
win32:LIBS += -lOpenGL32
      LIBS += -lGlU32
      LIBS +=User32.LIB
INCLUDEPATH += $$PWD/3rd/freetype/include
DEPENDPATH += $$PWD/3rd/freetype/include
# lib
win32-msvc2015:LIBS += $$PWD/3rd/GL/glut32.lib
win32-msvc2015:LIBS += -L$$PWD/3rd/STB_IMAGE/win32/ -lSTB_IMAGE
win32-msvc2015:LIBS += -L$$PWD/3rd/openni2/x86-Release/ -lOpenNI2
win32-msvc2015:LIBS += -L$$PWD/3rd/obpng/libs/win32/140/ -lobpng

win32: LIBS += -L$$PWD/3rd/opencv/lib/x86/vc14/lib/ -lopencv_world3410

INCLUDEPATH += $$PWD/3rd/opencv/lib/include
DEPENDPATH += $$PWD/3rd/opencv/lib/include

#INCLUDEPATH += $$PWD/3rd/directshow/Include
#INCLUDEPATH += $$PWD/3rd/directshow/qedit
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    res/image/oblog_little.ply

RESOURCES += \
    resource.qrc \
    shader.qrc
