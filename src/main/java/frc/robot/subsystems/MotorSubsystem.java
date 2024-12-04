package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor = new WPI_TalonSRX(0);
    private final PIDController pidController = new PIDController(0.0005, 0.0, 0.0);
    private final TalonSRXSimCollection motorSim = motor.getSimCollection();

    private static final int ENCODER_TICKS_PER_REV = 1024;
    private static final double ENCODER_TICKS_PER_DEGREE = ENCODER_TICKS_PER_REV / 360.0; //Update with encoder specs
    
    public void setMotorSpeed(double speed) {
        // Clamp speed between -1 and 1
        speed = MathUtil.clamp(speed, -1, 1); // Clamps the speed, no need to do min() and max()
        speed = MathUtil.applyDeadband(speed, 0.1); // Applies deadband, no need to do if/else statements
    
        motor.set(speed);
    }
    //Rotate to setpoint/angle methods//
    public MotorSubsystem() {
        motor.setNeutralMode(NeutralMode.Brake);
        pidController.setTolerance(1.0);
        motor.setSelectedSensorPosition(0); //Reset encoder at initialization (replace with limit switch)
    }

    public void rotateToAngle(double targetAngle) {
        double targetPosition = targetAngle * ENCODER_TICKS_PER_DEGREE;
        double currentPosition = motor.getSelectedSensorPosition();
        double pidOutput = pidController.calculate(currentPosition, targetPosition);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        motor.set(pidOutput);
    }

    public boolean isAtTarget() {
        return pidController.atSetpoint();
    }
    

    //Rotate at certain RPM (Revolutions per minute) methods//
    public void setMotorRPM(double targetRPM) {
        //Convert target RPM to encoder ticks per 100ms
        double targetTicksPer100ms = (targetRPM * ENCODER_TICKS_PER_REV) / 600.0;
        motor.set(ControlMode.Velocity, targetTicksPer100ms);
    }

    public double getCurrentRPM(){
        double currentTicksPer100ms = motor.getSelectedSensorVelocity(0);
        return (currentTicksPer100ms * 600.0) / ENCODER_TICKS_PER_REV;
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Position", motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Current RPM", getCurrentRPM());
    }

    @Override
    public void simulationPeriodic() {
         //Simulate encoder behavior
         double motorOutput = motorSim.getMotorOutputLeadVoltage() / RobotController.getBatteryVoltage();
         double simulatedTicksPer100ms = motorOutput * ENCODER_TICKS_PER_REV;
 
         //Update the simulated velocity and position
         motorSim.setQuadratureVelocity((int) simulatedTicksPer100ms);
         motorSim.addQuadraturePosition((int) simulatedTicksPer100ms);
    }
}
