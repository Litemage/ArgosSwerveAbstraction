/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the
///            terms of the license file in the root directory of this project.

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "argos_lib/config/cancoder_config.h"
#include "argos_lib/config/falcon_config.h"
#include "argos_lib/general/Framer.h"
#include "argos_lib/general/argos_imu.h"
#include "argos_lib/general/swerve_utils.h"
#include "ctre/Phoenix.h"
#include "frc/ADIS16448_IMU.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "utils/file_system_homing_storage.h"

namespace argos_lib::swerve {
/**
 * @brief A struct that contains all configuration options specific to modules
 * to pass to ArgosSwerveConfig class
 *
 */
struct ArgosModuleConfig {
  const u_char encoderAddress;
  const u_char driveMotorAddress;
  const u_char turnMotorAddress;
  const frc::Translation2d chassisOffset;
  const std::string_view canBus;  ///< CAN line on the CANivore to use
  constexpr ArgosModuleConfig(char encoderAddress, char driveMotorAddress,
                              char turnMotorAddress,
                              const frc::Translation2d& chassisOffset,
                              const std::string_view canBus)
      : encoderAddress{encoderAddress},
        driveMotorAddress{driveMotorAddress},
        turnMotorAddress{turnMotorAddress},
        chassisOffset{chassisOffset},
        canBus{canBus} {};
};

/**
 * @brief Wrapper for mk3 SwerveSpecialties modules with Falcon 500 motors
 *
 */
class ArgosModule {
 public:
  ArgosModule() = delete;
  ArgosModule(ArgosModule&&) = default;
  /**
   * @brief Construct a new Argos Module object
   *
   * @param config The configuration values for a the module
   */
  explicit ArgosModule(ArgosModuleConfig config)
      : m_encoder{config.encoderAddress, std::string(config.canBus)},
        m_drive{config.driveMotorAddress, std::string(config.canBus)},
        m_turn{config.turnMotorAddress, std::string(config.canBus)},
        m_chassisOffset{config.chassisOffset} {}
  CANCoder m_encoder;   ///< On-board encoder
  WPI_TalonFX m_drive;  ///< Drive motor
  WPI_TalonFX m_turn;   ///< Turn motor
  const frc::Translation2d
      m_chassisOffset;  ///< Module's offset from initial center of rotation
};

/**
 * @brief A wrapper for input velocities
 * vX is percent forward on the interval [-1, 1]
 * vY is percent left on the interval [-1, 1]
 * vOmega is percent rotational +CCW on the interval [-1, 1]
 *
 */
struct Velocities {
  const double vX;
  const double vY;
  const double vOmega;
};

/**
 * @brief A wrapper class for all the modules on a robot. Class in development,
 * subject to change
 *
 */
template <int N>
class ArgosSwerveConfig {
 public:
  ArgosSwerveConfig() = delete;

  /**
   * @brief Construct a new Argos Swerve Config object containing all the
   * necessary config to run the drivetrain
   *
   * @param turnEncoderResolution The resolution of the encoder on the turn
   * motor
   * @param homesPath The path to the file containing the home values
   * @param maxVelocity The maximum velocity for determing maximum change
   * velocity during optimization
   * @param wheel The first ArgosModuleConfig object
   * @param wheels The rest of the module configuration objects
   */
  template <typename... Wheels>
  explicit ArgosSwerveConfig(
      const double turnEncoderResolution, const std::string& homesPath,
      const units::velocity::feet_per_second_t maxVelocity,
      const ArgosModuleConfig& wheel, const Wheels&... wheels)
      : m_homesPath{homesPath},
        m_turnConversionFact{360.0 / turnEncoderResolution},
        m_maxVelocity{maxVelocity},
        m_moduleConfigs{wheel, wheels...},
        m_chasisOffsets{wheel.chassisOffset, wheels.chassisOffset...} {}

  /**
   * @brief Returns a wpi::array<frc::Translation2d, N> containing all of the
   * chasis offsets in the same order as they were constructed
   *
   * @return wpi::array<frc::Translation2d, N>
   */
  inline wpi::array<frc::Translation2d, N> GetChassisOffsets() const {
    return m_chasisOffsets;
  }

  /**
   * @brief Get the array of module configuration objects
   *
   * @return const wpi::array<ArgosModuleConfig, N>&
   */
  const wpi::array<ArgosModuleConfig, N>& GetConfigs() const {
    return m_moduleConfigs;
  }

  /**
   * @brief Get the path to the homes file
   *
   * @return const std::string
   */
  const std::string GetHomesPath() const { return m_homesPath; }

  /**
   * @brief Gets the conversion factor for the encoder on the turn motor, in
   * degrees per sensor unit
   *
   * @return double
   */
  double GetTurnConversionFactor() const { return m_turnConversionFact; }

  /**
   * @brief Get the max velocity of the drivetrain during optimization
   *
   * @return units::velocity::feet_per_second_t
   */
  units::velocity::feet_per_second_t GetMaxVelocity() const {
    return m_maxVelocity;
  }

 private:
  std::string m_homesPath;
  const double
      m_turnConversionFact;  ///< The conversion factor for the encoder on the
                             ///< turn motor, in degrees per sensor unit
  const units::velocity::feet_per_second_t
      m_maxVelocity;  ///< The maximum velocity to determing max change velocity
                      ///< in Argos optimization function
  wpi::array<ArgosModuleConfig, N>
      m_moduleConfigs;  ///< The array holding configuration for each module in
                        ///< the drivetrain
  wpi::array<frc::Translation2d, N>
      m_chasisOffsets;  ///< Array of Tranlation2d objects representing the
                        ///< modules' offset from the center of rotation
};

/**
 * @brief enum containing control modes.
 *
 */
enum class SwerveControlMode { FIELD_CENTRIC, ROBOT_CENTRIC };

/**
 * @brief Class containing basic functions of a drivetrain to build off of
 *
 */
template <int N, typename IMU>
class ArgosSwerve {
 public:
  ArgosSwerve() = delete;

  /**
   * @brief Construct a new Argos swerve object
   *
   *
   *
   * @param config Configuration containing all the swerve modules
   * @param imu Pointer to an ArgosIMU
   * @param instance The instance of robot, (Competition? Practice?)
   * @param controlMode The control mode to initialize to
   */
  ArgosSwerve(const ArgosSwerveConfig<N>& config, ArgosIMU<IMU>* imu,
              argos_lib::RobotInstance instance,
              SwerveControlMode controlMode = SwerveControlMode::FIELD_CENTRIC)
      : m_config{config},
        m_modules{m_config.GetConfigs().begin(), m_config.GetConfigs().end()},
        m_swerveKinematics(m_config.GetChassisOffsets()),
        m_instance{instance},
        m_controlMode(controlMode),
        m_pImu(imu),
        m_fsStorage{m_config.GetHomesPath()} {}

 protected:
  /**
   * @brief Get raw module states based off a set of desired velocities
   *
   * @param velocities given velocities in respective directions on the interval
   * [-1, 1]. Follows WPIlib reference frame (subject to change)
   * @return wpi::array<frc::SwerveModuleState, N>
   */
  wpi::array<frc::SwerveModuleState, N> GetRawModuleStates(
      Velocities velocities);
  /**
   * @brief Get the current angle of the robot relative to the zeroed angle on
   * the field (subject to change)
   *
   * @return units::angle::degree_t
   */
  inline units::angle::degree_t GetFieldCentricAngle() const {
    return m_pImu->GetAngle() - m_fieldCentricHome;
  }
  /**
   * @brief Homes to a particular angle on the field (subject to change)
   *
   * @param homeAngle The angle the robot is homing too
   */
  inline void FieldHome(units::degree_t homeAngle) {
    m_fieldCentricHome = m_pImu->GetAngle() - homeAngle;
  }

  /**
   * @brief Set the Control Mode object
   *
   * @param controlMode The control scheme to use
   */
  inline void SetControlMode(ControlMode controlMode) {
    m_controlMode = controlMode;
  }

  /**
   * @brief Sames saves the current drivetrain home to file system
   *
   * @param virtualAngle The angle that the robot is being homed too. Usually 0
   */
  virtual void HomeDrivetrainToFS(units::degree_t virtualAngle);

  /**
   * @brief Saves new homes, resets the imu, and immediately initializes motors;
   * NOTE: intended to be a utility function for debugging / library
   * development.
   *
   * @param virtualAngle
   */
  virtual void Home(units::degree_t virtualAngle);

  /**
   * @brief Reset the IMU to zero
   *
   */
  virtual void ResetIMU() { m_pImu->Reset(); }

  /**
   * @brief Initializes encoders on start-up with saved homes
   *
   * @return True -> Initialization succeeded False -> Initialization failed
   */
  virtual bool InitDrivetrainHomes();

  /**
   * @brief Sets the field home offset to later calculate a relative angle
   * between the robot and the field.
   *
   * @param virtualAngle The angle relative to the field to home to
   */
  void HomeFieldCentric(units::degree_t virtualAngle);

  /**
   * @brief Gets the angle given in sensor units of the encoder resolution of
   * the modules
   *
   * @param degrees The value to convert to sensor units
   * @return const units::degree_t The angle in sensor units
   */
  double ToSensorUnit(const units::degree_t degrees) {
    return degrees.to<double>() / m_config.GetTurnConversionFactor();
  }
  /**
   * @brief Converts the given sensor units to degrees
   *
   * @return const units::degree_t The sensor unit angle in degrees
   */
  units::degree_t ToAngle(const double sensorunit) {
    return units::make_unit<units::degree_t>(
        m_config.GetTurnConversionFactor() * sensorunit);
  }
  /**
   * @brief Drive the robot using just velocities
   *
   * @param inputVelocities The velocities to apply to the drivetrain.
   */
  void Drive(Velocities inputVelocities);

  /**
   * @brief Stops the drivetrain by setting all the modules to zero.
   *
   */
  void StopDrivetrain();

  /**
   * @brief Describes module devices, used for configuration
   *
   */
  enum class ModuleDevice { Turn = 0, Drive, Encoder };

  template <typename Competition, typename Practice>
  void ConfigModuleDevice(unsigned char moduleIndex, ModuleDevice dev,
                          units::time::millisecond_t timeout = 100_ms) {
    switch (dev) {
      case ModuleDevice::Turn:
        argos_lib::falcon_config::FalconConfig<Competition, Practice>(
            m_modules[moduleIndex].m_turn, timeout, m_instance);
        break;
      case ModuleDevice::Drive:
        argos_lib::falcon_config::FalconConfig<Competition, Practice>(
            m_modules[moduleIndex].m_drive, timeout, m_instance);
        break;
      case ModuleDevice::Encoder:
        argos_lib::cancoder_config::CanCoderConfig<Competition, Practice>(
            m_modules[moduleIndex].m_encoder, timeout, m_instance);
        break;
    }
  }

  template <typename Competition, typename Practice>
  void ConfigAllModuleDevice(ModuleDevice dev,
                             units::time::millisecond_t timeout = 100_ms) {
    switch (dev) {
      case ModuleDevice::Turn:
        for (unsigned int i = 0; i < m_modules.size(); i++) {
          argos_lib::falcon_config::FalconConfig<Competition, Practice>(
              m_modules[i].m_turn, timeout, m_instance);
        }
        break;
      case ModuleDevice::Drive:
        for (unsigned int i = 0; i < m_modules.size(); i++) {
          argos_lib::falcon_config::FalconConfig<Competition, Practice>(
              m_modules[i].m_drive, timeout, m_instance);
        }
        break;
      case ModuleDevice::Encoder:
        for (unsigned int i = 0; i < m_modules.size(); i++) {
          argos_lib::cancoder_config::CanCoderConfig<Competition, Practice>(
              m_modules[i].m_encoder, timeout, m_instance);
        }
        break;
    }
  }

 private:
  ArgosSwerveConfig<N>
      m_config;  ///< object containing configuration from ArgosSwerveConfig to
                 ///< configure the drivetrain
  std::vector<ArgosModule> m_modules;  ///< vector containing all the modules
                                       ///< being used in this drivetrain
  frc::SwerveDriveKinematics<N>
      m_swerveKinematics;  ///< Kinematic model for the swerve drivetrain
  argos_lib::RobotInstance
      m_instance;  ///< Robot instance. (Competition or Practice)
  SwerveControlMode m_controlMode;  ///< the current control mode of the robot
                                    ///< (Robot-centric or Field-centric)
  ArgosIMU<IMU>* m_pImu;  ///< Pointer to the IMU being used with the drivetrain
  FileSystemHomingStorage
      m_fsStorage;  ///< Object that handles saving and loading homes from files
  units::degree_t
      m_fieldCentricHome;  ///< the difference between the real IMU angle and
                           ///< our actual field-relative angle.
  frc::ChassisSpeeds m_emptySpeeds = frc::ChassisSpeeds{
      units::make_unit<units::velocity::meters_per_second_t>(0),
      units::make_unit<units::velocity::meters_per_second_t>(0),
      units::make_unit<units::angular_velocity::radians_per_second_t>(
          0)};  ///< empty speeds to be used when needing to return 0 velocities
};

}  // namespace argos_lib::swerve

#include "../../../cpp/general/argos_swerve.inc"
