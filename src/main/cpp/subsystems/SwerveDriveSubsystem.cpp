// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDriveSubsystem.h"

#include <string>

#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
using argos_lib::falcon_config::FalconConfig;
using argos_lib::swerve::ArgosSwerveConfig;

ArgosSwerveConfig config =
    ArgosSwerveConfig<4>{4096,
                         std::string("/homes/swerveHomes"),
                         12_fps,
                         swerveModulesBeta::fLModule,
                         swerveModulesBeta::frModule,
                         swerveModulesBeta::brModule,
                         swerveModulesBeta::bLModule};

SwerveDriveSubsystem::SwerveDriveSubsystem()
    : argos_lib::swerve::ArgosSwerve<4, ctre::phoenix::sensors::Pigeon2>(
          config, &m_pigeon, argos_lib::RobotInstance::Competition,
          argos_lib::swerve::SwerveControlMode::FIELD_CENTRIC){
  /* ―――――――――――――――――― Configure all drive motors ―――――――――――――――――― */
  /* Note: remember that the indexes are in the order in which the modules are
   * supplied in the configuration */
  ConfigAllModuleDevice<motorConfigs::genericDrive, motorConfigs::genericDrive>(
      ModuleDevice::Drive);
  /* ――――――――――――――――――――― Configure turn motors ―――――――――――――――――――― */
  ConfigModuleDevice<motorConfigs::frontLeftTurn, motorConfigs::frontLeftTurn>(
      0, ModuleDevice::Turn);
  ConfigModuleDevice<motorConfigs::frontRightTurn,
                     motorConfigs::frontRightTurn>(1, ModuleDevice::Turn);
  ConfigModuleDevice<motorConfigs::backRightTurn, motorConfigs::backRightTurn>(
      2, ModuleDevice::Turn);
  ConfigModuleDevice<motorConfigs::backLeftTurn, motorConfigs::backLeftTurn>(
      3, ModuleDevice::Turn);
  /* ―――――――――――――――――――――― Configure encoders ―――――――――――――――――――――― */
  ConfigModuleDevice<motorConfigs::frontLeftTurn, motorConfigs::frontLeftTurn>(
      0, ModuleDevice::Encoder);
  ConfigModuleDevice<motorConfigs::frontRightTurn,
                     motorConfigs::frontRightTurn>(1, ModuleDevice::Encoder);
  ConfigModuleDevice<motorConfigs::backRightTurn, motorConfigs::backRightTurn>(
      2, ModuleDevice::Encoder);
  ConfigModuleDevice<motorConfigs::backLeftTurn, motorConfigs::backLeftTurn>(
      3, ModuleDevice::Encoder);

  /* ―――――――――――――――――――― Initialize homes/motors ――――――――――――――――――― */
  InitializeFromHomes();
}

// This method will be called once per scheduler run
void SwerveDriveSubsystem::Periodic() {}

void SwerveDriveSubsystem::Disable() {
  StopDrivetrain();
  // rest of disabling done here
}

void SwerveDriveSubsystem::SwerveDrive(double forward, double left,
                                       double rotation) {
  if (!m_drivetrainInitialized) {
    printf("DISABLING ROBOT\n");
    frc::SmartDashboard::PutBoolean("DRIVETRAIN INIT", m_drivetrainInitialized);
    Disable();
    return;
  }
  frc::SmartDashboard::PutNumber("DRIVE FORWARD", forward);
  frc::SmartDashboard::PutNumber("DRIVE LEFT", left);
  frc::SmartDashboard::PutNumber("DRIVE ROTATION", rotation);
  // Necessary conversion into WPI reference frame before going into Drive()
  Drive(argos_lib::swerve::Velocities{forward, left, rotation});
}

void SwerveDriveSubsystem::Home(units::degree_t virtualAngle) {
  HomeDrivetrainToFS(virtualAngle);
}

void SwerveDriveSubsystem::ResetIMU() { ResetIMU(); }

void SwerveDriveSubsystem::InitializeFromHomes() {
  printf("INITIALIZING FROM HOMES");
  m_drivetrainInitialized = InitDrivetrainHomes();
}
