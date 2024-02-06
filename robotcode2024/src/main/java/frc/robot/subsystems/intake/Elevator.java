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
    private final CANSparkMax elev;
    private final CANSparkMax intake;

    private SparkPIDController elevController;
    private RelativeEncoder elevRelativeEncoder;

    private SparkPIDController intakeController;
    private RelativeEncoder intakeRelativeEncoder;

    private boolean intakePIDEnabled;
    private double intakeSetpoint;

    private boolean elevPIDEnabled;
    private double elevSetpoint;

    private double elevPower;

    private double intakePower;

    Elevator() {
        super();

        elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
        intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);

        elev.restoreFactoryDefaults();
        intake.restoreFactoryDefaults();

        elev.setIdleMode(IdleMode.kBrake);
        elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

        intake.setIdleMode(IdleMode.kBrake);
        intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

        intakeController = intake.getPIDController();
        intakeRelativeEncoder = intake.getEncoder();
        intakeRelativeEncoder.setPositionConversionFactor(Constants.IntakeConstants.intakeGR);

        elevController = elev.getPIDController();
        elevRelativeEncoder = elev.getEncoder();
        elevRelativeEncoder.setPositionConversionFactor(Constants.ElevatorConstants.elevPosConv);

        // set PID coefficients
        elevController.setP(Constants.ElevatorConstants.kP);
        elevController.setI(Constants.ElevatorConstants.kI);
        elevController.setD(Constants.ElevatorConstants.kD);
        elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

        intakeController.setP(Constants.IntakeConstants.kP);
        intakeController.setI(Constants.IntakeConstants.kI);
        intakeController.setD(Constants.IntakeConstants.kD);
        intakeController.setOutputRange(Constants.IntakeConstants.kMinOutput, Constants.IntakeConstants.kMaxOutput);

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

        intakeController.setSmartMotionMaxVelocity(Constants.IntakeConstants.maxVel, smartMotionSlot);
        intakeController.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.minVel, smartMotionSlot);
        intakeController.setSmartMotionMaxAccel(Constants.IntakeConstants.maxAcc, smartMotionSlot);
        intakeController.setSmartMotionAllowedClosedLoopError(Constants.IntakeConstants.allowedErr, smartMotionSlot);

        elev.burnFlash();
        intake.burnFlash();
    }

    public double getElevPos() {
        return elevRelativeEncoder.getPosition();
    }

    public void moveElev(double motorElevPower) {
        if (getElevPos() >= Constants.ElevatorConstants.maxHeight - Constants.ElevatorConstants.safeZone) {
            motorElevPower = 0;
        }
        if (getElevPos() <= Constants.ElevatorConstants.minHeight + Constants.ElevatorConstants.safeZone) {
            motorElevPower = 0;
        }

        if (Math.abs(motorElevPower) < 0.05) {
            elevPIDEnabled = true;
        } else {
            elevPower = motorElevPower;
            elev.set(elevPower);
            elevSetpoint = getElevPos();
            elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kSmartMotion);
            elevPIDEnabled = false;
        }
    }

    public double getIntakePos() {
        return intakeRelativeEncoder.getPosition();
    }

    public void moveIntake(double motorIntakePower) {
        if (getIntakePos() >= Constants.IntakeConstants.maxAngle - Constants.IntakeConstants.safeZone) {
            motorIntakePower = 0;
        }
        if (getIntakePos() <= Constants.IntakeConstants.minAngle + Constants.IntakeConstants.safeZone) {
            motorIntakePower = 0;
        }

        if (Math.abs(motorIntakePower) < 0.05) {
            intakePIDEnabled = true;
        } else {
            intakePower = motorIntakePower;
            intake.set(intakePower);
            intakeSetpoint = getIntakePos();
            intakeController.setReference(intakeSetpoint, CANSparkMax.ControlType.kSmartMotion);
            intakePIDEnabled = false;
        }
    }

    public void setIntakePosition(IntakePosition position) {
        elevSetpoint = position.getIntakeHeight();
        intakeSetpoint = position.getIntakeAngle();

        elevPIDEnabled = true;
        intakePIDEnabled = true;
    }

    public static class IntakePosition {
        private double elevPos;
        private double intakeAng;

        public IntakePosition(double elevPos, double intakeAng) {
            this.elevPos = elevPos;
            this.intakeAng = intakeAng;
        }

        // height from bottom of carriage
        public double getIntakeHeight() {
            return elevPos;
        }

        public double getIntakeAngle() {
            return intakeAng;
        }
    }

    @Override
    public void periodic() {
        if (elevPIDEnabled) {
            elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kSmartMotion);
        }
        if (intakePIDEnabled) {
            intakeController.setReference(intakeSetpoint, CANSparkMax.ControlType.kSmartMotion);
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
