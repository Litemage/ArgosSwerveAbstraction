/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the
///            terms of the license file in the root directory of this project.
#pragma once

#include "ctre/phoenix/sensors/Pigeon2.h"
#include "frc/ADIS16448_IMU.h"
#include "units/angle.h"

using ctre::phoenix::sensors::AxisDirection;
using ctre::phoenix::sensors::Pigeon2;

// TODO does this namespace make sense?
namespace argos_lib::swerve {

enum ArgosAxis {
  PositiveZ,
  PositiveY,
  PositiveX,
  NegativeZ,
  NegativeY,
  NegativeX
};

template <typename T>
class ArgosIMU {
 public:
  explicit ArgosIMU(T *imu, ArgosAxis upAxis, ArgosAxis forwardAxis)
      : m_imu{imu}, m_upAxis{upAxis}, m_forwardAxis{forwardAxis} {
    Configure();
  }
  void Configure() { /* by default, do nothing */
  }
  units::degree_t GetAngle();
  void Reset() { /* By default, do nothing */
  }
  T *GetInstance() { return m_imu; }

 private:
  T *m_imu;
  ArgosAxis m_upAxis;
  ArgosAxis m_forwardAxis;
};

/* ―――――――――――――――――――――― Configure the imus ―――――――――――――――――――――― */

template <>
inline void ArgosIMU<frc::ADIS16448_IMU>::Configure() {
  // we dont' care about forward axis for AD imu
  frc::ADIS16448_IMU *imu = m_imu;
  switch (m_upAxis) {
    case ArgosAxis::PositiveX:
      imu->SetYawAxis(frc::ADIS16448_IMU::kX);
      break;
    case ArgosAxis::PositiveY:
      imu->SetYawAxis(frc::ADIS16448_IMU::kY);
      break;
    case ArgosAxis::PositiveZ:
      imu->SetYawAxis(frc::ADIS16448_IMU::kZ);
      break;
    // There are no negative axis available, this will be handled later, in
    // getting the yaw axis
    case ArgosAxis::NegativeX:
      imu->SetYawAxis(frc::ADIS16448_IMU::kX);
      break;
    case ArgosAxis::NegativeY:
      imu->SetYawAxis(frc::ADIS16448_IMU::kY);
      break;
    case ArgosAxis::NegativeZ:
      imu->SetYawAxis(frc::ADIS16448_IMU::kZ);
      break;
  }
}

template <>
inline void ArgosIMU<Pigeon2>::Configure() {
  Pigeon2 *imu = m_imu;
  // Look mom, no giant switch statement :D
  AxisDirection upAxis = (AxisDirection)m_upAxis;
  AxisDirection forwardAxis = (AxisDirection)m_forwardAxis;

  imu->ConfigMountPose(forwardAxis, upAxis);
}

/* ――――――――――――――――― Get the value of the yaw axis ―――――――――――――――― */
template <>
inline units::degree_t ArgosIMU<frc::ADIS16448_IMU>::GetAngle() {
  frc::ADIS16448_IMU *imu = m_imu;
  units::degree_t currentAngle{imu->GetAngle()};
  // Handle that negative axis ambiguity here
  if ((char)m_upAxis > 2) {
    currentAngle * -1;
  }
  return currentAngle;
}

template <>
inline units::degree_t ArgosIMU<Pigeon2>::GetAngle() {
  Pigeon2 *imu = m_imu;
  units::degree_t currentAngle{imu->GetYaw()};
  return currentAngle;
}

/* ―――――――――――――――― Handle resetting IMU ――――――――――――――― */
template <>
inline void ArgosIMU<frc::ADIS16448_IMU>::Reset() {
  frc::ADIS16448_IMU *imu = m_imu;
  imu->Reset();
}

template <>
inline void ArgosIMU<Pigeon2>::Reset() {
  Pigeon2 *imu = m_imu;
  imu->SetYaw(0);
}

/* ―――――― Allow consumers to get the respective imu instance ―――――― */

}  // namespace argos_lib::swerve