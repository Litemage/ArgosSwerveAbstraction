// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/RunCommand.h"

// 0 is driver controller address, and 1 is operator controller address
RobotContainer::RobotContainer() : m_swerveDrive{}, m_controllers{0, 1} {
  // Initialize all of your commands and subsystems here
  // Initialize from homes
  m_swerveDrive.Home(0_deg);
  /* ―――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――― */
  /*                     Drivetrain Defalt Command                    */
  /* ―――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――――― */
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        /* ――――――――――――――――――――― Get Controller inputs ―――――――――――――――――――― */
        double vY, vX, vR;
        // Forward velocity
        // Inverted
        vY = -m_controllers.DriverController().GetY(
            argos_lib::XboxController::JoystickHand::kLeftHand);
        // Sideways velocity
        // Inverted
        vX = -m_controllers.DriverController().GetX(
            argos_lib::XboxController::JoystickHand::kLeftHand);
        // Rotational velocity
        // Inverted
        vR = -m_controllers.DriverController().GetX(
            argos_lib::XboxController::JoystickHand::kRightHand);

        /* ――― filter for dead area of controller (robot go brr if not) ――― */
        if (abs(vY) < 0.05) {
          vY = 0;
        }
        if (abs(vX) < 0.05) {
          vX = 0;
        }
        if (abs(vR) < 0.05) {
          vR = 0;
        }

        /* ――――――――――――――――――――― Actually Drive Robot ――――――――――――――――――――― */
        m_swerveDrive.SwerveDrive(vY, vX, vR);
      },
      {&m_swerveDrive}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // TODO do not keep this here
  return nullptr;
}
