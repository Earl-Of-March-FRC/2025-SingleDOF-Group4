package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateContinouosly extends Command {
    private final MotorSubsystem motorSubsystem;
    private final DoubleSupplier speedSupplier;

    public RotateContinouosly(MotorSubsystem motorSubsystem, DoubleSupplier speedSupplier) {
        this.motorSubsystem = motorSubsystem;
        this.speedSupplier = speedSupplier;

        addRequirements(motorSubsystem);
    }

    @Override
    public void execute() {
        motorSubsystem.setMotorSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stopMotor(); // Stop motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously
    }
}
