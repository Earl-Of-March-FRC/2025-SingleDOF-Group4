// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RotateAtSpeed;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final MotorSubsystem motorSubsystem = new MotorSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
      m_driverController.x().onTrue(new RotateToAngleCommand(motorSubsystem, 90.0)); // Rotate to 90 degrees 
      m_driverController.a().onTrue(new RotateAtSpeed(motorSubsystem, 3000)); // Set to 3000 RPM on A button
      m_driverController.b().onTrue(new RotateAtSpeed(motorSubsystem, 0));    // Stop motor on B button   
  }

  public Command getAutonomousCommand() {
      return null;
  }
}
