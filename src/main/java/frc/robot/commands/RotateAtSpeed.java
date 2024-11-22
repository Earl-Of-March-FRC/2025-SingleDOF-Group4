// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class RotateAtSpeed extends Command {

  private final MotorSubsystem motorSubsystem;
  private final double targetRPM;

  public RotateAtSpeed(MotorSubsystem msub, double tRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetRPM = tRPM;
    motorSubsystem = msub;

    addRequirements(motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      motorSubsystem.setMotorRPM(targetRPM);
  }

  public void end(boolean interrupted) {
    motorSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
