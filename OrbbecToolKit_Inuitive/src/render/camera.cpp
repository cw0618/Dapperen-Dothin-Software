﻿#include "camera.h"

const float CAMERA_STEP = 0.1;

Camera::Camera() 
{
}


void Camera::forward()
{
  _position[2] += CAMERA_STEP;
  emit zPosChanged(_position[2]);
  _notify();
}


void Camera::backward()
{
  _position[2] -= CAMERA_STEP;
  emit zPosChanged(_position[2]);
  _notify();
}


void Camera::left()
{
  _position[0] += CAMERA_STEP;
  emit xPosChanged(_position[0]);
  _notify();
}


void Camera::right()
{
  _position[0] -= CAMERA_STEP;
  emit xPosChanged(_position[0]);
  _notify();

}


void Camera::up()
{
  _position[1] -= CAMERA_STEP;
  emit yPosChanged(_position[1]);
  _notify();
}


void Camera::down()
{
  _position[1] += CAMERA_STEP;
  emit yPosChanged(_position[1]);
  _notify();

}


void Camera::setFrontCPDistance(double distance) {
  _frontClippingPlaneDistance = distance; 
  _notify();
}


void Camera::setRearCPDistance(double distance) {
  _rearClippingDistance = distance;
  _notify();
}


void Camera::setPosition(const QVector3D& position) {
  _position = position;
}


void Camera::setXRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != _xRotation) {
    _xRotation = angle;
    emit xRotationChanged(angle);
    _notify();
  }
}


void Camera::setYRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != _yRotation) {
    _yRotation = angle;
    emit yRotationChanged(angle);
    _notify();
  }
}


void Camera::setZRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != _zRotation) {
    _zRotation = angle;
    emit zRotationChanged(angle);
    _notify();
  }
}

void Camera::setXPosition(float pos){
    _position[0] = pos;
    _notify();
}

void Camera::setYPosition(float pos) {
    _position[1] = pos;
    _notify();
}

void Camera::setZPosition(float pos) {
    _position[2] = pos;
    _notify();
}


void Camera::rotate(int dx, int dy, int dz) {
  setXRotation(_xRotation + dx);
  setYRotation(_yRotation + dy);
  setZRotation(_zRotation + dz);
}


CameraState Camera::state() const {
  return CameraState(
    _position,
    QVector3D((float)_xRotation/RK, (float)_yRotation/RK, (float)_zRotation/RK),
    _frontClippingPlaneDistance,
    _rearClippingDistance
    );
}
