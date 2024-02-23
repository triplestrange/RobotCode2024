package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rails extends SubsystemBase {

    private final CANSparkMax lWinch;
    private final CANSparkMax rWinch;

    private SparkPIDController lWinchController;
    private RelativeEncoder lWinchEncoder;

    private SparkPIDController rWinchController;
    private RelativeEncoder rWinchEncoder;

    private boolean rWinchPIDEnabled;
    private double rWinchSetpoint;

    private boolean lWinchPIDEnabled;
    private double lWinchSetpoint;

    private double lWinchPower;

    private double rWinchPower;

    public Rails() {
        super();

        lWinch = new CANSparkMax(Constants.CAN.CLIMBL, MotorType.kBrushless);
        rWinch = new CANSparkMax(Constants.CAN.CLIMBR, MotorType.kBrushless);

        lWinch.restoreFactoryDefaults();
        rWinch.restoreFactoryDefaults();

        lWinch.setIdleMode(IdleMode.kBrake);
        lWinch.setSmartCurrentLimit(Constants.ELECTRICAL.climbCurrentLimit);

        rWinch.setIdleMode(IdleMode.kBrake);
        rWinch.setSmartCurrentLimit(Constants.ELECTRICAL.climbCurrentLimit);

        lWinchController = lWinch.getPIDController();
        lWinchEncoder = lWinch.getEncoder();
        lWinchEncoder.setPositionConversionFactor(Constants.ClimbConstants.climbPosConv);

        rWinchController = rWinch.getPIDController();
        rWinchEncoder = rWinch.getEncoder();
        rWinchEncoder.setPositionConversionFactor(Constants.ClimbConstants.climbPosConv);

        // set PID coefficients
        lWinchController.setP(Constants.ClimbConstants.kP);
        lWinchController.setI(Constants.ClimbConstants.kI);
        lWinchController.setD(Constants.ClimbConstants.kD);
        lWinchController.setOutputRange(Constants.ClimbConstants.kMinOutput, Constants.ClimbConstants.kMaxOutput);

        rWinchController.setP(Constants.ClimbConstants.kP);
        rWinchController.setI(Constants.ClimbConstants.kI);
        rWinchController.setD(Constants.ClimbConstants.kD);
        rWinchController.setOutputRange(Constants.ClimbConstants.kMinOutput, Constants.ClimbConstants.kMaxOutput);

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
        lWinchController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, smartMotionSlot);
        lWinchController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, smartMotionSlot);
        lWinchController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, smartMotionSlot);
        lWinchController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, smartMotionSlot);

        rWinchController.setSmartMotionMaxVelocity(Constants.IntakeConstants.maxVel, smartMotionSlot);
        rWinchController.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.minVel, smartMotionSlot);
        rWinchController.setSmartMotionMaxAccel(Constants.IntakeConstants.maxAcc, smartMotionSlot);
        rWinchController.setSmartMotionAllowedClosedLoopError(Constants.IntakeConstants.allowedErr, smartMotionSlot);

        lWinch.burnFlash();
        rWinch.burnFlash();
    }

    public double getLWinchPos() {
        return lWinchEncoder.getPosition();
    }

    public double getRWinchPos() {
        return rWinchEncoder.getPosition();
    }

    public void moveClimb(double motorLWinchPower, double motorRWinchPower) {
        if (getLWinchPos() >= Constants.ClimbConstants.maxHeight - Constants.ClimbConstants.safeZone) {
            motorLWinchPower = 0;
        }
        if (getLWinchPos() <= Constants.ClimbConstants.minHeight + Constants.ClimbConstants.safeZone) {
            motorLWinchPower = 0;
        }

        if (Math.abs(motorLWinchPower) < 0.05) {
            lWinchPIDEnabled = true;
        } else {
            lWinchPower = motorLWinchPower;
            lWinch.set(lWinchPower);
            lWinchSetpoint = getLWinchPos();
            lWinchController.setReference(getLWinchPos(), CANSparkMax.ControlType.kSmartMotion);
            lWinchPIDEnabled = false;
        }

        if (getRWinchPos() >= Constants.ClimbConstants.maxHeight - Constants.ClimbConstants.safeZone) {
            motorRWinchPower = 0;
        }
        if (getRWinchPos() <= Constants.ClimbConstants.minHeight + Constants.ClimbConstants.safeZone) {
            motorRWinchPower = 0;
        }

        if (Math.abs(motorRWinchPower) < 0.05) {
            rWinchPIDEnabled = true;
        } else {
            rWinchPower = motorRWinchPower;
            rWinch.set(rWinchPower);
            rWinchSetpoint = getRWinchPos();
            lWinchController.setReference(getRWinchPos(), CANSparkMax.ControlType.kSmartMotion);
            lWinchPIDEnabled = false;
        }
    }

    public void setClimbPosition(ClimbPosition position) {
        lWinchSetpoint = position.getLeftClimb();
        rWinchSetpoint = position.getRightClimb();

        lWinchPIDEnabled = true;
        rWinchPIDEnabled = true;
    }

    public static class ClimbPosition {
        private double lWinchPos;
        private double rWinchPos;

        public ClimbPosition(double lWinchPos, double rWinchPos) {
            this.lWinchPos = lWinchPos;
            this.rWinchPos = rWinchPos;
        }

        // height from bottom of carriage
        public double getLeftClimb() {
            return lWinchPos;
        }

        public double getRightClimb() {
            return rWinchPos;
        }
    }

    @Override
    public void periodic() {
        if (lWinchPIDEnabled) {
            lWinchController.setReference(lWinchSetpoint, CANSparkMax.ControlType.kSmartMotion);
        }
        if (rWinchPIDEnabled) {
            rWinchController.setReference(rWinchSetpoint, CANSparkMax.ControlType.kSmartMotion);
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

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", Constants.IntakeConstants.kP);
        SmartDashboard.putNumber("I Gain", Constants.IntakeConstants.kI);
        SmartDashboard.putNumber("D Gain", Constants.IntakeConstants.kD);
        SmartDashboard.putNumber("Max Output", Constants.IntakeConstants.kMaxOutput);
        SmartDashboard.putNumber("Min Output", Constants.IntakeConstants.kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", Constants.IntakeConstants.maxVel);
        SmartDashboard.putNumber("Min Velocity", Constants.IntakeConstants.minVel);
        SmartDashboard.putNumber("Max Acceleration", Constants.IntakeConstants.maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", Constants.IntakeConstants.allowedErr);
    }
}
