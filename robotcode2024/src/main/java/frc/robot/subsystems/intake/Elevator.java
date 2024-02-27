package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elev;
    private final CANSparkMax intake;

    private SparkPIDController elevController;
    private RelativeEncoder elevRelativeEncoder;

    private ProfiledPIDController intakeController;
    private DutyCycleEncoder intakeEncoder;

    private boolean intakePIDEnabled;
    public double intakeSetpoint;

    private boolean elevPIDEnabled;
    public double elevSetpoint;

    private double elevPower;

    private double intakePower;

    /**
     * Creates a new Elevator.
     */
    public Elevator() {
        super();

        elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
        intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);

        elev.restoreFactoryDefaults();
        intake.restoreFactoryDefaults();

intake.setInverted(true);
        elev.setIdleMode(IdleMode.kBrake);
        elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

        intake.setIdleMode(IdleMode.kBrake);
        intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

        intakeController = new ProfiledPIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI,
                Constants.IntakeConstants.kD,
                new Constraints(Constants.IntakeConstants.kMaxAngularSpeedMetersPerSecond, Constants.IntakeConstants.kMaxAngularAccelerationMetersPerSecondSquared));
        intakeEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.intakeAbsInput);

        elevController = elev.getPIDController();
        elevRelativeEncoder = elev.getEncoder();
        elevRelativeEncoder.setPositionConversionFactor(Constants.ElevatorConstants.elevPosConv);



        // set PID coefficients
        elevController.setP(Constants.ElevatorConstants.kP);
        elevController.setI(Constants.ElevatorConstants.kI);
        elevController.setD(Constants.ElevatorConstants.kD);
        elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

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

        elev.burnFlash();
        intake.burnFlash();
    }

    public double getElevPos() {
        return elevRelativeEncoder.getPosition();
    }

    public void moveElev(double motorElevPower, double motorIntakePower) {
        // if (getElevPos() >= Constants.ElevatorConstants.maxHeight - Constants.ElevatorConstants.safeZone) {
        //     motorElevPower = 0;
        // }
        // if (getElevPos() <= Constants.ElevatorConstants.minHeight + Constants.ElevatorConstants.safeZone) {
        //     motorElevPower = 0;
        // }

        if (Math.abs(motorElevPower) < 0.05) {
            elevPIDEnabled = true;
        } else {
            elevPower = motorElevPower;
           // System.out.println(elevPower);
            // elev.set(elevPower);
            elevSetpoint = getElevPos();
           // elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kSmartMotion);
            elevPIDEnabled = false;
        }

        if ((getIntakePos() >= Constants.IntakeConstants.maxAngle - Constants.IntakeConstants.safeZone) && motorIntakePower > 0) {
            motorIntakePower = 0;
        }
        if (getIntakePos() <= Constants.IntakeConstants.minAngle + Constants.IntakeConstants.safeZone && motorIntakePower < 0) {
            motorIntakePower = 0;
        }

        if (Math.abs(motorIntakePower) < 0.05) {
            intakePIDEnabled = true;
        } else {
            intakePower = motorIntakePower;
            // intake.set(intakePower);
            intakeSetpoint = getIntakePos();
            intakeController.reset(intakeSetpoint);
            intakePIDEnabled = false;
        }
    }

    public double getIntakePos() {
        return MathUtil.inputModulus(-intakeEncoder.getAbsolutePosition() * 180 - Constants.IntakeConstants.intakeAbsOffset, -160, 20);
    }

    public void setIntakePosition(IntakePosition position) {
        elevSetpoint = position.getIntakeHeight();
        intakeSetpoint = position.getIntakeAngle();

        elevPIDEnabled = true;
        intakePIDEnabled = true;
    }

    public void resetPIDs() {
        intakeSetpoint = getIntakePos();
        elevSetpoint = getElevPos();
        intakeController.reset(intakeSetpoint);
        elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kPosition);
        intakePIDEnabled = true;
        elevPIDEnabled = true;
    
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
            elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kPosition);
        }
        if (intakePIDEnabled) {
            intakePower = intakeController.calculate(getIntakePos(), intakeSetpoint);
        }
        if (!intakeEncoder.isConnected()) {
            intakePower = 0;
        }

        intake.set(intakePower);
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("degree", getIntakePos());
        SmartDashboard.putBoolean("Is Encoder Plugged", intakeEncoder.isConnected());
        SmartDashboard.putNumber("angle setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Power", intakePower);


    }
}
