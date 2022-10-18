// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>

#include <string>

#include "argos_lib/general/argos_swerve.h"

using argos_lib::swerve::ArgosModuleConfig;

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace address {
constexpr const char frontLeftDrive = 1;
constexpr const char frontLeftTurn = 2;
constexpr const char frontRightDrive = 3;
constexpr const char frontRightTurn = 4;
constexpr const char backRightDrive = 5;
constexpr const char backRightTurn = 6;
constexpr const char backLeftDrive = 7;
constexpr const char backLeftTurn = 8;
constexpr const char frontLeftEncoder = 1;
constexpr const char frontRightEncoder = 2;
constexpr const char backRightEncoder = 3;
constexpr const char backLeftEncoder = 4;
}  // namespace address

namespace motorConfigs {
struct genericDrive {
  constexpr static auto inverted =
      ctre::phoenix::motorcontrol::InvertType::None;
  constexpr static bool sensorPhase = false;
  constexpr static auto neutralDeadband = 0.001;
  constexpr static auto neutralMode =
      ctre::phoenix::motorcontrol::NeutralMode::Coast;
  constexpr static auto voltCompSat = 11.0_V;
  constexpr static auto statusFrameMotorMode =
      argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
  constexpr static auto pid0_selectedSensor =
      ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
  constexpr static auto pid0_kP = 0.11;
  constexpr static auto pid0_kI = 0.0;
  constexpr static auto pid0_kD = 0.0;
  constexpr static auto pid0_kF = 0.05;
  constexpr static auto pid0_iZone = 500.0;
  constexpr static auto pid0_allowableError = 0.0;
};
struct frontLeftTurn {
  constexpr static auto inverted =
      ctre::phoenix::motorcontrol::InvertType::None;
  constexpr static bool sensorPhase = false;
  constexpr static auto neutralDeadband = 0.001;
  constexpr static auto neutralMode =
      ctre::phoenix::motorcontrol::NeutralMode::Brake;
  constexpr static auto voltCompSat = 11.0_V;
  constexpr static auto statusFrameMotorMode =
      argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
  constexpr static auto remoteFilter0_addr = address::frontLeftEncoder;
  constexpr static auto remoteFilter0_type = ctre::phoenix::motorcontrol::
      RemoteSensorSource::RemoteSensorSource_CANCoder;
  constexpr static auto pid0_selectedSensor =
      ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
  constexpr static auto pid0_kP = 1.4;
  constexpr static auto pid0_kI = 0.0005;
  constexpr static auto pid0_kD = 0.0;
  constexpr static auto pid0_kF = 0.0;
  constexpr static auto pid0_iZone = 500.0;
  constexpr static auto pid0_allowableError = 0.0;
};
struct frontRightTurn {
  constexpr static auto inverted =
      ctre::phoenix::motorcontrol::InvertType::None;
  constexpr static bool sensorPhase = false;
  constexpr static auto neutralDeadband = 0.001;
  constexpr static auto neutralMode =
      ctre::phoenix::motorcontrol::NeutralMode::Brake;
  constexpr static auto voltCompSat = 11.0_V;
  constexpr static auto statusFrameMotorMode =
      argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
  constexpr static auto remoteFilter0_addr = address::frontRightEncoder;
  constexpr static auto remoteFilter0_type = ctre::phoenix::motorcontrol::
      RemoteSensorSource::RemoteSensorSource_CANCoder;
  constexpr static auto pid0_selectedSensor =
      ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
  constexpr static auto pid0_kP = 1.4;
  constexpr static auto pid0_kI = 0.0005;
  constexpr static auto pid0_kD = 0.0;
  constexpr static auto pid0_kF = 0.0;
  constexpr static auto pid0_iZone = 500.0;
  constexpr static auto pid0_allowableError = 0.0;
};
struct backRightTurn {
  constexpr static auto inverted =
      ctre::phoenix::motorcontrol::InvertType::None;
  constexpr static bool sensorPhase = false;
  constexpr static auto neutralDeadband = 0.001;
  constexpr static auto neutralMode =
      ctre::phoenix::motorcontrol::NeutralMode::Brake;
  constexpr static auto voltCompSat = 11.0_V;
  constexpr static auto statusFrameMotorMode =
      argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
  constexpr static auto remoteFilter0_addr = address::backRightEncoder;
  constexpr static auto remoteFilter0_type = ctre::phoenix::motorcontrol::
      RemoteSensorSource::RemoteSensorSource_CANCoder;
  constexpr static auto pid0_selectedSensor =
      ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
  constexpr static auto pid0_kP = 1.4;
  constexpr static auto pid0_kI = 0.0005;
  constexpr static auto pid0_kD = 0.0;
  constexpr static auto pid0_kF = 0.0;
  constexpr static auto pid0_iZone = 500.0;
  constexpr static auto pid0_allowableError = 0.0;
};
struct backLeftTurn {
  constexpr static auto inverted =
      ctre::phoenix::motorcontrol::InvertType::None;
  constexpr static bool sensorPhase = false;
  constexpr static auto neutralDeadband = 0.001;
  constexpr static auto neutralMode =
      ctre::phoenix::motorcontrol::NeutralMode::Brake;
  constexpr static auto voltCompSat = 11.0_V;
  constexpr static auto statusFrameMotorMode =
      argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
  constexpr static auto remoteFilter0_addr = address::backLeftEncoder;
  constexpr static auto remoteFilter0_type = ctre::phoenix::motorcontrol::
      RemoteSensorSource::RemoteSensorSource_CANCoder;
  constexpr static auto pid0_selectedSensor =
      ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
  constexpr static auto pid0_kP = 1.4;
  constexpr static auto pid0_kI = 0.0005;
  constexpr static auto pid0_kD = 0.0;
  constexpr static auto pid0_kF = 0.0;
  constexpr static auto pid0_iZone = 500.0;
  constexpr static auto pid0_allowableError = 0.0;
};
}  // namespace motorConfigs

namespace swerve_offsets {
constexpr auto frontLeftLOffset = 4.0_in;
constexpr auto frontLeftWOffset = 4.0_in;
constexpr auto frontRightLOffset = 4.0_in;
constexpr auto frontRightWOffset = 4.0_in;
constexpr auto backRightWOffset = 4.0_in;
constexpr auto backRightLOffset = 4.0_in;
constexpr auto backLeftWOffset = 4.0_in;
constexpr auto backLeftLOffset = 4.0_in;
}  // namespace swerve_offsets

namespace chassis {
constexpr units::inch_t width{28.0};
constexpr units::inch_t length{31.0};
}  // namespace chassis

namespace measure_up {
const frc::Translation2d frontLeft =
    frc::Translation2d(chassis::length / 2 - swerve_offsets::frontLeftLOffset,
                       chassis::width / 2 - swerve_offsets::frontLeftWOffset);
const frc::Translation2d frontRight =
    frc::Translation2d(chassis::length / 2 - swerve_offsets::frontRightLOffset,
                       -chassis::width / 2 + swerve_offsets::frontRightWOffset);
const frc::Translation2d backRight =
    frc::Translation2d(-chassis::length / 2 + swerve_offsets::backRightLOffset,
                       -chassis::width / 2 + swerve_offsets::backRightWOffset);
const frc::Translation2d backLeft =
    frc::Translation2d(-chassis::length / 2 + swerve_offsets::backLeftLOffset,
                       chassis::width / 2 - swerve_offsets::backLeftWOffset);
}  // namespace measure_up

namespace swerveModulesBeta {
const ArgosModuleConfig fLModule{
    address::frontLeftEncoder, address::frontLeftDrive, address::frontLeftTurn,
    measure_up::frontLeft, "drive"};
const ArgosModuleConfig frModule{
    address::frontRightEncoder, address::frontRightDrive,
    address::frontRightTurn, measure_up::frontRight, "drive"};
const ArgosModuleConfig bLModule{address::backLeftEncoder,
                                 address::backLeftDrive, address::backLeftTurn,
                                 measure_up::backLeft, "drive"};
const ArgosModuleConfig brModule{
    address::backRightEncoder, address::backRightDrive, address::backRightTurn,
    measure_up::backRight, "drive"};
}  // namespace swerveModulesBeta

const ArgosModuleConfig frModule{1, 2, 3, frc::Translation2d{12_in, -10_in},
                                 "drive"};
