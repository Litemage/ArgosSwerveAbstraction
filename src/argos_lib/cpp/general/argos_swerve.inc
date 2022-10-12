/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the
///            terms of the license file in the root directory of this project.

#pragma once

#include <vector>

#include "argos_lib/general/argos_swerve.h"
#include "argos_lib/general/swerve_utils.h"
#include "units/angle.h"

namespace argos_lib::swerve {
/* ―――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――― */
/*                   Argos Swerve Member Functions                  */
/* ―――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――― */
template <int N, typename IMU>
wpi::array<frc::SwerveModuleState, N> ArgosSwerve<N, IMU>::GetRawModuleStates(
    Velocities velocities) {
  // IF SPEEDS ZERO, SET MOTORS TO ZERO AND RETURN
  if (velocities.vX == 0 && velocities.vY == 0 && velocities.vOmega == 0) {
    StopDrivetrain();
    return m_swerveKinematics.ToSwerveModuleStates(m_emptySpeeds);
  }
  switch (m_controlMode) {
    case SwerveControlMode::FIELD_CENTRIC: {
      frc::ChassisSpeeds fieldCentricSpeeds =
          frc::ChassisSpeeds::FromFieldRelativeSpeeds(
              units::make_unit<units::meters_per_second_t>(velocities.vX),
              units::make_unit<units::meters_per_second_t>(velocities.vY),
              units::make_unit<units::angular_velocity::radians_per_second_t>(
                  velocities.vOmega),
              frc::Rotation2d(GetFieldCentricAngle()));

      // Return speeds to consumer
      return m_swerveKinematics.ToSwerveModuleStates(fieldCentricSpeeds);
    }
    case SwerveControlMode::ROBOT_CENTRIC: {
      frc::ChassisSpeeds robotCentricSpeeds{
          units::make_unit<units::velocity::meters_per_second_t>(velocities.vX),
          units::make_unit<units::velocity::meters_per_second_t>(velocities.vY),
          units::make_unit<units::angular_velocity::radians_per_second_t>(
              velocities.vOmega)};

      return m_swerveKinematics.ToSwerveModuleStates(robotCentricSpeeds);
    }
    default: {
      return m_swerveKinematics.ToSwerveModuleStates(m_emptySpeeds);
    }
  }
  return m_swerveKinematics.ToSwerveModuleStates(m_emptySpeeds);
}

template <int N, typename IMU>
void ArgosSwerve<N, IMU>::HomeDrivetrainToFS(units::degree_t virtualAngle) {
  std::vector<units::degree_t> moduleHomes;
  for (uint i = 0; i < m_modules.size(); i++) {
    moduleHomes.push_back(units::make_unit<units::angle::degree_t>(
                              m_modules[i].m_encoder.GetAbsolutePosition()) -
                          virtualAngle);
  }

  m_fsStorage.SaveList(moduleHomes);
}

template <int N, typename IMU>
bool ArgosSwerve<N, IMU>::InitDrivetrainHomes() {
  std::optional<std::vector<units::degree_t>> modulePositions =
      m_fsStorage.LoadList(N);
  if (!modulePositions) {
    // ALERT HERE THAT THERE ARE NO VALUES, BUT FOR NOW, JUST PRINT
    std::printf("%d HEY NO SAVED VALUES IN std::FILE SYSTEM!!!!", __LINE__);
    return false;
  }
  for (uint i = 0; i < m_modules.size(); i++) {
    // Hardware - Saved home value gives relative position in terms of saved
    // home
    units::degree_t encoderRelativePos = units::make_unit<units::degree_t>(
        m_modules[i].m_encoder.GetAbsolutePosition() -
        modulePositions.value()[i].to<double>());
    // Constrain angle
    encoderRelativePos = ConstrainAngle(encoderRelativePos, 0_deg, 360_deg);
    // Set position to motor
    m_modules[i].m_encoder.SetPosition(encoderRelativePos.to<double>());
  }
  return true;
}

template <int N, typename IMU>
void ArgosSwerve<N, IMU>::HomeFieldCentric(units::degree_t virtualAngle) {
  // GetAngle() negated because IMU board is physically inverted, causing
  // positive Z to become CW+
  m_fieldCentricHome =
      // Calculate home with current hardware value - virtual angle
      -m_pImu->GetAngle() - virtualAngle;
}

template <int N, typename IMU>
void ArgosSwerve<N, IMU>::Drive(Velocities inputVelocities) {
  if (inputVelocities.vX == 0 && inputVelocities.vY == 0 &&
      inputVelocities.vOmega == 0) {
    StopDrivetrain();
  }

  wpi::array<frc::SwerveModuleState, N> moduleStates =
      GetRawModuleStates(inputVelocities);
  for (int i = 0; i < N; i++) {
    moduleStates.at(i) = argos_lib::swerve::Optimize(
        moduleStates.at(i),
        ToAngle(m_modules[i].m_turn.GetSelectedSensorPosition()), 0_rpm, 0_fps,
        m_config.GetMaxVelocity());
  }
  for (int i = 0; i < N; i++) {
    m_modules[i].m_drive.Set(
        ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput,
        double(moduleStates.at(i).speed));
    m_modules[i].m_turn.Set(
        ctre::phoenix::motorcontrol::TalonFXControlMode::Position,
        ToSensorUnit(moduleStates.at(i).angle.Degrees()));
  }
}

template <int N, typename IMU>
void ArgosSwerve<N, IMU>::StopDrivetrain() {
  for (int i = 0; i < N; i++) {
    m_modules[i].m_drive.Set(0.0);
    m_modules[i].m_turn.Set(0.0);
  }
}
}  // namespace argos_lib::swerve
