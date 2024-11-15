package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final WPI_VictorSPX motor = new WPI_VictorSPX(0);
    private final Encoder encoder = new Encoder(0, 1);
    private final PIDController pidController = new PIDController(0.05, 0.0, 0.0);

    // Conversion factor for encoder ticks to degrees
    private static final double ENCODER_TICKS_PER_DEGREE = 4096 / 360.0; // Update with your encoder specs

    public MotorSubsystem() {
        motor.setNeutralMode(NeutralMode.Brake);
        pidController.setTolerance(1.0); // 1 degree tolerance for setpoint
        encoder.reset(); // Reset encoder at initialization
    }

    public void rotateToAngle(double targetAngle) {
        double targetPosition = targetAngle * ENCODER_TICKS_PER_DEGREE;
        double currentPosition = encoder.getDistance();
        double pidOutput = pidController.calculate(currentPosition, targetPosition);
        motor.set(pidOutput);
    }

    public boolean isAtTarget() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        // Run this periodically, typically for logging
        System.out.println("Current Position: " + encoder.getDistance());
    }
}
