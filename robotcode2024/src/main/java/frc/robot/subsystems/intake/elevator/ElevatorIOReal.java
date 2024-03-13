package frc.robot.subsystems.intake.elevator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.elevator.Elevator.IntakePosition;

public class ElevatorIOReal implements ElevatorIO {
  private CANSparkMax elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
  private CANSparkMax intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);
  private SparkPIDController elevController;
  private ProfiledPIDController intakeController;
  private RelativeEncoder elevRelativeEncoder = elev.getEncoder();
  private DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.intakeAbsInput);

  private boolean intakePIDEnabled;
  public double intakeSetpoint;

  private boolean elevPIDEnabled;
  public double elevSetpoint;

  private double elevPower;

  private double intakePower;

  private double inputVolts = 0.0;

  public ElevatorIOReal() {

    intake.setInverted(true);
    elev.setInverted(false);

    elev.setIdleMode(IdleMode.kBrake);
    elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

    elevRelativeEncoder = elev.getEncoder();

    elevRelativeEncoder.setPositionConversionFactor(Constants.ElevatorConstants.elevPosConv);

    elevController = elev.getPIDController();

    elevRelativeEncoder.setPosition(0);

    intakeController = new ProfiledPIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI,
        Constants.IntakeConstants.kD,
        new Constraints(Constants.IntakeConstants.kMaxAngularSpeedMetersPerSecond,
            Constants.IntakeConstants.kMaxAngularAccelerationMetersPerSecondSquared));

    // set PID coefficients
    elevController.setP(Constants.ElevatorConstants.kP);
    elevController.setI(Constants.ElevatorConstants.kI);
    elevController.setD(Constants.ElevatorConstants.kD);
    elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

    elevController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, 0);
    elevController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, 0);
    elevController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, 0);
    elevController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, 0);

    elevSetpoint = getElevPos();

    elev.burnFlash();
    intake.burnFlash();
  }

  public double getElevPos() {
    return elevRelativeEncoder.getPosition();
  }

  public double getIntakePos() {
    return MathUtil.inputModulus(
        -intakeEncoder.getAbsolutePosition() * 180 - Constants.IntakeConstants.intakeAbsOffset, -160, 20);
  }

  public void resetPIDs() {
    intakeSetpoint = getIntakePos();
    elevSetpoint = getElevPos();
    intakeController.reset(intakeSetpoint);
    elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kPosition);
    intakePIDEnabled = true;
    elevPIDEnabled = true;

  }

  @Override
  public void moveElev(double motorElevPower, double motorIntakePower) {
    if (Math.abs(motorElevPower) < 0.05) {
      elevPIDEnabled = true;
    } else {
      elevPower = motorElevPower;
      elev.set(elevPower);
      elevSetpoint = getElevPos();
      elevPIDEnabled = false;
    }
    if (Math.abs(motorIntakePower) < 0.05) {
      intakePIDEnabled = true;
    } else {
      intakePower = motorIntakePower;
      intakeSetpoint = getIntakePos();
      intakeController.reset(intakeSetpoint);
      intakePIDEnabled = false;
    }
  }

  @Override
  public void setElev(IntakePosition position) {
    elevSetpoint = position.getIntakeHeight();
    intakeSetpoint = position.getIntakeAngle();

    elevPIDEnabled = true;
    intakePIDEnabled = true;
  }

  @Override
  public void setIdleMode(IdleMode elevIdleMode, IdleMode intakIdleMode) {
    elev.setIdleMode(elevIdleMode);
    intake.setIdleMode(intakIdleMode);
  }

  @Override
  public void setVoltage(double volts) {
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
  }
}
