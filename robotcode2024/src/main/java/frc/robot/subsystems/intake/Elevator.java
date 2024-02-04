package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public final CANSparkMax elev;

    public double elevPov;

    public SparkPIDController elevController;
    public RelativeEncoder elevRelativeEncoder;

    private boolean elevPIDEnabled;
    private double elevSetPoint;

    // public boolean hasSetPoint;
    Elevator() {
        elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
        elev.setIdleMode(IdleMode.kBrake);
        elev.restoreFactoryDefaults();
        elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

        elevController = elev.getPIDController();
        elevRelativeEncoder = elev.getEncoder();
        elevRelativeEncoder.setPositionConversionFactor(Constants.ElevatorConstants.elevPosConv);

        // set PID coefficients
        elevController.setP(Constants.ElevatorConstants.kP);
        elevController.setI(Constants.ElevatorConstants.kI);
        elevController.setD(Constants.ElevatorConstants.kD);
        elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

        // hasSetPoint = false;

        /**
         * Smart Motion coefficients are set on a SparkPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        elevController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, smartMotionSlot);
        elevController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, smartMotionSlot);
        elevController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, smartMotionSlot);
        elevController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, smartMotionSlot);
    }

    public double getPos() {
        return elevRelativeEncoder.getPosition();
    }

    public void moveElev(double motorElevPower) {
        if (getPos() >= Constants.ElevatorConstants.maxHeight - Constants.ElevatorConstants.safeZone) {
            motorElevPower = 0;
        }
        if (getPos() <= Constants.ElevatorConstants.minHeight + Constants.ElevatorConstants.safeZone) {
            motorElevPower = 0;
        }

        if (Math.abs(motorElevPower) < 0.05) {
            elevPIDEnabled = true;
        } else {
            elevPower = motorElevShoulder;
            shoulderJoint.set(shoulderPower);
            shoulderSetpoint = getShoulder();
            shoulderPID.reset(shoulderSetpoint);
            shoulderPIDEnabled = false;
        }
    }

    public void periodic() {
        if (elevPIDEnabled) {
            elevController.setReference(elevSetPoint, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    public void updateSmartDashBoard() {

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", Constants.ElevatorConstants.kP);
        SmartDashboard.putNumber("I Gain", Constants.ElevatorConstants.kI);
        SmartDashboard.putNumber("D Gain", Constants.ElevatorConstants.kD);
        SmartDashboard.putNumber("Max Output", Constants.ElevatorConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", Constants.ElevatorConstants.kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", Constants.ElevatorConstants.maxVel);
        SmartDashboard.putNumber("Min Velocity", Constants.ElevatorConstants.minVel);
        SmartDashboard.putNumber("Max Acceleration", Constants.ElevatorConstants.maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", Constants.ElevatorConstants.allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

    }
}
