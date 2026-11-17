#ifndef ADAFRUIT_MPU6050_COMPONENT_H
#define ADAFRUIT_MPU6050_COMPONENT_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "component_interface.h"

class AdafruitMPU6050Sensor : public ComponentInterface {
  private:
    Adafruit_MPU6050 _mpu;
    uint8_t _i2c_address;
    
    sensors_event_t _a, _g, _temp;
    float _accel[3]; // [m/s^2]
    float _gyro[3];  // [rad/s]

  public:
    AdafruitMPU6050Sensor(uint8_t i2c_address = 0x68, uint8_t ID = 0)
      : ComponentInterface(ID), _i2c_address(i2c_address) {
      for (int i = 0; i < 3; i++) {
        _accel[i] = 0.0f;
        _gyro[i] = 0.0f;
      }
    }

    bool init() override {
      Wire.begin();
      if (!_mpu.begin(_i2c_address)) {
        return false;
      }

      _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

      return true;
    }

    void read() {
      _mpu.getEvent(&_a, &_g, &_temp);
      
      _accel[0] = _a.acceleration.x;
      _accel[1] = _a.acceleration.y;
      _accel[2] = _a.acceleration.z;

      _gyro[0] = _g.gyro.x;
      _gyro[1] = _g.gyro.y;
      _gyro[2] = _g.gyro.z;
    }

    inline float getAccelX() const { return _accel[0]; }
    inline float getAccelY() const { return _accel[1]; }
    inline float getAccelZ() const { return _accel[2]; }

    inline float getGyroX() const { return _gyro[0]; }
    inline float getGyroY() const { return _gyro[1]; }
    inline float getGyroZ() const { return _gyro[2]; }
};

#endif // ADAFRUIT_MPU6050_COMPONENT_H
