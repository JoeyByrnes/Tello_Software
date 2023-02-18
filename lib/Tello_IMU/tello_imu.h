#include "../lib/Eigen/Dense"
#include "vn/sensors.h"

class telloIMU {
public:
    // Constructors
    telloIMU() {}
    telloIMU(Eigen::Quaterniond orientation,
             Eigen::Vector3d position,
             Eigen::Vector3d acceleration,
             Eigen::Vector3d angularVelocity,
             vn::sensors::VnSensor* vs)
    : Orientation(orientation),
      Position(position),
      Acceleration(acceleration),
      AngularVelocity(angularVelocity),
      vs(vs) {}

    // Fields
    Eigen::Quaterniond Orientation;
    Eigen::Vector3d Position;
    Eigen::Vector3d Acceleration;
    Eigen::Vector3d AngularVelocity;
    vn::sensors::VnSensor* vs;

    // Functions
    void Initialize();
    void SetupSerial();

    // Getters and Setters
    Eigen::Quaterniond getOrientation() { return Orientation; }
    void setOrientation(Eigen::Quaterniond orientation) { Orientation = orientation; }

    Eigen::Vector3d getPosition() { return Position; }
    void setPosition(Eigen::Vector3d position) { Position = position; }

    Eigen::Vector3d getAcceleration() { return Acceleration; }
    void setAcceleration(Eigen::Vector3d acceleration) { Acceleration = acceleration; }

    Eigen::Vector3d getAngularVelocity() { return AngularVelocity; }
    void setAngularVelocity(Eigen::Vector3d angularVelocity) { AngularVelocity = angularVelocity; }

    vn::sensors::VnSensor* getVs() { return vs; }
    void setVs(vn::sensors::VnSensor* vs) { telloIMU::vs = vs; }
};

void telloIMU::Initialize() {
    // Initialize the IMU sensor here
}

void telloIMU::SetupSerial() {
    // Setup the serial communication here
}