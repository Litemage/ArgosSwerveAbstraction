// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "argos_lib/general/Framer.h"
#include "argos_lib/general/argos_imu.h"
#include "argos_lib/general/argos_swerve.h"
#include "units/angle.h"

using argos_lib::swerve::ArgosAxis;
using argos_lib::swerve::ArgosIMU;

class SwerveDriveSubsystem
    : public frc2::SubsystemBase,
      protected argos_lib::swerve::ArgosSwerve<4, frc::ADIS16448_IMU> {
 public:
  SwerveDriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Disable();

  void SwerveDrive(double forward, double left, double rotation);

  void Home(units::degree_t virtualAngle);

  void InitializeFromHomes();

  void ResetIMU();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::ADIS16448_IMU adisIMU =
      frc::ADIS16448_IMU{frc::ADIS16448_IMU::kZ, frc::SPI::Port::kMXP,
                         frc::ADIS16448_IMU::CalibrationTime::_8s};

  ArgosIMU<frc::ADIS16448_IMU> m_imu = ArgosIMU<frc::ADIS16448_IMU>(
      frc::ADIS16448_IMU{frc::ADIS16448_IMU::kZ, frc::SPI::Port::kMXP,
                         frc::ADIS16448_IMU::CalibrationTime::_8s},
      ArgosAxis::NegativeZ, ArgosAxis::NegativeX);

  Framer::RefFrame m_drivetrainFrame;
  bool m_drivetrainInitialized = false;
};
