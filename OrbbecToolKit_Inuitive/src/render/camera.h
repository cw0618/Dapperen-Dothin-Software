#pragma once

#include <QObject>
#include <QVector3D>

struct CameraState 
{
  CameraState(const QVector3D& position_, const QVector3D& rotation_,
              double frontClippingDistance_, double farClippingDistance_)
    : position(position_),
      rotation(rotation_),
      frontClippingDistance(frontClippingDistance_),
      rearClippingDistance(farClippingDistance_)
  {}

  const QVector3D position;
  const QVector3D rotation;
  const double frontClippingDistance;
  const double rearClippingDistance;
};


class Camera : public QObject
{
  Q_OBJECT

public:
  enum RotationSTEP {RK = 1};

  Camera();

  void forward();
  void backward();
  void left();
  void right();
  void up();
  void down();
  void setPosition(const QVector3D& position);

  void rotate(int dx, int dy, int dz);

  void setFrontCPDistance(double distance);
  void setRearCPDistance(double distance);

  CameraState state() const;


signals:
  void changed(const CameraState& newState);
  void xRotationChanged(int angle);
  void yRotationChanged(int angle);
  void zRotationChanged(int angle);

  void xPosChanged(float angle);
  void yPosChanged(float angle);
  void zPosChanged(float angle);


public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setXPosition(float pos);
    void setYPosition(float pos);
    void setZPosition(float pos);


private:
    double _frontClippingPlaneDistance{ 0.f };
    double _rearClippingDistance{ 0.f };
    QVector3D _position;
    int _xRotation{ 0 };
    int _yRotation{ 0 };
    int _zRotation{ 0 };

    void _notify() {emit changed(state());}
};

