// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "argos_lib/general/Framer.h"
#include "argos_lib/general/argos_swerve.h"
#include "units/angle.h"

class SwerveDriveSubsystem : public frc2::SubsystemBase,
                             protected argos_lib::swerve::ArgosSwerve<4> {
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
  frc::ADIS16448_IMU m_imu =
      frc::ADIS16448_IMU(frc::ADIS16448_IMU::IMUAxis::kZ, frc::SPI::Port::kMXP,
                         frc::ADIS16448_IMU::CalibrationTime::_8s);

  Framer::RefFrame m_drivetrainFrame;
  bool m_drivetrainInitialized = false;
};
