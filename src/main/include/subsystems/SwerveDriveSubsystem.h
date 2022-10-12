// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "argos_lib/general/argos_imu.h"
#include "argos_lib/general/argos_swerve.h"
#include "units/angle.h"

using argos_lib::swerve::ArgosAxis;
using argos_lib::swerve::ArgosIMU;

class SwerveDriveSubsystem : public frc2::SubsystemBase,
                             protected argos_lib::swerve::ArgosSwerve<
                                 4, ctre::phoenix::sensors::Pigeon2> {
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

  ctre::phoenix::sensors::Pigeon2 pigeon{1, "drive"};

  ArgosIMU<ctre::phoenix::sensors::Pigeon2> m_pigeon{
      &pigeon, ArgosAxis::PositiveZ, ArgosAxis::PositiveY};

  bool m_drivetrainInitialized = false;
};
