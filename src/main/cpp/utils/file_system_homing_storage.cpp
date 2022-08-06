/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/file_system_homing_storage.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include "wpi/fs.h"

FileSystemHomingStorage::FileSystemHomingStorage(const fs::path& swerveHomesPath)
    : m_swerveHomesPath{swerveHomesPath} {}

bool FileSystemHomingStorage::Save(const argos_lib::swerve::SwerveModulePositions& homePosition) {
  try {
    std::ofstream configFile(GetFilePath(), std::ios::out);
    configFile << homePosition.FrontLeft.to<double>() << ' ' << homePosition.FrontRight.to<double>() << ' '
               << homePosition.RearRight.to<double>() << ' ' << homePosition.RearLeft.to<double>();
    configFile.close();
    return true;
  } catch (...) {
    // Error accessing file
    std::cout << "[ERROR] Could not write to config file\n";
    return false;
  }
}

// TODO make sure that trailing whitespace makes no difference in loading/saving home values
bool FileSystemHomingStorage::SaveList(const std::vector<units::angle::degree_t>& homePositions) {
  try {
    std::ofstream configFile(GetFilePath(), std::ios::out);
    configFile << static_cast<double>(homePositions.size()) << ' ';
    for (units::angle::degree_t position : homePositions) {
      configFile << position.to<double>() << ' ';
    }
    configFile.close();
    return true;
  } catch (...) {
    std::cout << "[ERROR] Could not write to config file\n";
    return false;
  }
}

std::optional<argos_lib::swerve::SwerveModulePositions> FileSystemHomingStorage::Load() {
  try {
    std::ifstream configFile(GetFilePath(), std::ios::in);
    double frontLeft, frontRight, rearRight, rearLeft;
    configFile >> frontLeft >> frontRight >> rearRight >> rearLeft;
    configFile.close();

    return argos_lib::swerve::SwerveModulePositions{{units::make_unit<units::degree_t>(frontLeft)},
                                                    {units::make_unit<units::degree_t>(frontRight)},
                                                    {units::make_unit<units::degree_t>(rearRight)},
                                                    {units::make_unit<units::degree_t>(rearLeft)}};
  } catch (...) {
    // Error accessing file
    std::cout << "[ERROR] Could not read from config file\n";
    return std::nullopt;
  }
}

std::optional<std::vector<units::angle::degree_t>> FileSystemHomingStorage::LoadList(int numModules) {
  try {
    std::ifstream configFile(GetFilePath(), std::ios::in);
    double size;
    configFile >> size;
    if (size != numModules) {
      std::cout << "[ERROR] Number of modules in file doesn't match number of requested modules\n";
      return std::nullopt;
    }
    std::optional<std::vector<units::angle::degree_t>> positions;
    for (int i = 0; i < size; i++) {
      double position;
      configFile >> position;
      positions->push_back(units::make_unit<units::angle::degree_t>(position));
    }
    return positions;
  } catch (...) {
    // Error accessing file
    std::cout << "[ERROR] Could not read from config file\n";
    return std::nullopt;
  }
}

fs::path FileSystemHomingStorage::GetFilePath() {
  static const fs::path homeDir{"/home/lvuser"};
  static const fs::path configFile{homeDir / m_swerveHomesPath};

  // Create empty file if it doesn't exist yet
  if (!fs::exists(configFile)) {
    fs::create_directories(configFile.parent_path());
    std::ofstream newFile(configFile);
    newFile.close();
  }

  return configFile;
}
