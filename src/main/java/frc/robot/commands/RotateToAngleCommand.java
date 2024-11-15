
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class RotateToAngleCommand extends Command {

    private final MotorSubsystem motorSubsystem;
    private final double targetAngle;

    public RotateToAngleCommand(MotorSubsystem motorSubsystem, double targetAngle) {
        this.motorSubsystem = motorSubsystem;
        this.targetAngle = targetAngle;

        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("RotateToAngleCommand started. Target Angle: " + targetAngle);
    }

    @Override
    public void execute() {
        motorSubsystem.rotateToAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return motorSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stopMotor();
        if (interrupted) {
            System.out.println("RotateToAngleCommand interrupted!");
        } else {
            System.out.println("RotateToAngleCommand finished!");
        }
    }
}
