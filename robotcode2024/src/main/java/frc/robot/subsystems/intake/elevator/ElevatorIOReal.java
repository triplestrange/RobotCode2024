package frc.robot.subsystems.intake.elevator;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.FaultID;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.elevator.Elevator.IntakePosition;

public class ElevatorIOReal implements ElevatorIO {
  public CANSparkMax elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);

  public SparkPIDController elevController = elev.getPIDController();

  public RelativeEncoder elevRelativeEncoder = elev.getEncoder();
  public DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.intakeAbsInput);

  private double winchInput;
  private double intakeInputVolts;
  private double intakeInputSpeed;

  private double offset;

  public ElevatorIOReal() {

    intake.setInverted(true);
    elev.setInverted(false);

    elev.setIdleMode(IdleMode.kBrake);
    elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

    elevController.setP(Constants.ElevatorConstants.kP);
    elevController.setI(Constants.ElevatorConstants.kI);
    elevController.setD(Constants.ElevatorConstants.kD);
    elevController.setOutputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);

    elevController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, 0);
    elevController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, 0);
    elevController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, 0);
    elevController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, 0);

    elev.burnFlash();
    intake.burnFlash();

    elevRelativeEncoder.setPositionConversionFactor(Constants.ElevatorConstants.elevPosConv);
    elevRelativeEncoder.setVelocityConversionFactor(
        Constants.ElevatorConstants.elevDrumRadiusMeters * Constants.ElevatorConstants.elevSimPosConv / 60);
    elevRelativeEncoder.setPosition(0);

    double winchMotorCurrent = elev.getOutputCurrent();
    double winchAppliedVolts = elev.getAppliedOutput() * elev.getBusVoltage();

    double elevatorPosMeters = Units.inchesToMeters(elevRelativeEncoder.getPosition());
    double elevatorVelMetersPerSecond = elevRelativeEncoder.getVelocity();

    double jointMotorCurrent = intake.getOutputCurrent();
    double jointAppliedVolts = intake.getAppliedOutput() * elev.getBusVoltage();

    double jointPosDeg = MathUtil.inputModulus(
        -intakeEncoder.getAbsolutePosition() * 180 - Constants.IntakeConstants.intakeAbsOffset - offset, -160, 20);
    double joinVelDeg = intake.getEncoder().getVelocity();

    double elevTempCelcius = elev.getMotorTemperature();
    double jointTempCelcius = intake.getMotorTemperature();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.jointAbsoluteEncoderConnected = intakeEncoder.isConnected();

    inputs.elevMotorConnected = elev.getFault(FaultID.kSensorFault);

    inputs.elevatorPosMeters = elevRelativeEncoder.getPosition();
    inputs.elevatorVelMetersPerSecond = elevRelativeEncoder.getVelocity();
    inputs.winchAppliedVolts = elev.getAppliedOutput() * elev.getBusVoltage();
    inputs.winchMotorCurrent = elev.getOutputCurrent();
    inputs.winchTempCelcius = elev.getMotorTemperature();
    inputs.winchInputVolts = winchInput;

    inputs.jointPosDeg = MathUtil.inputModulus(
        -intakeEncoder.getAbsolutePosition() * 180 - Constants.IntakeConstants.intakeAbsOffset - offset, -160, 20);
    inputs.jointAppliedVolts = intake.getAppliedOutput() * intake.getBusVoltage();
    inputs.jointMotorCurrent = intake.getOutputCurrent();
    inputs.jointTempCelcius = intake.getMotorTemperature();
    inputs.jointInputVolts = intakeInputVolts;
    inputs.jointInputSpeed = intakeInputSpeed;

  }

  @Override
  public void runHeightSetpoint(double height) {
    elevController.setReference(height, CANSparkMax.ControlType.kPosition);

  }

  @Override
  public void runWinchVolts(double elevVolts) {
    winchInput = elevVolts;
    elev.setVoltage(winchInput);
  }

  // @Override
  // public void runJointVolts(double intakeVolts) {
  //   intakeInputVolts = intakeVolts;
  //   intake.setVoltage(intakeInputVolts);
  // }
  @Override
  public void runJointPower(double jointPower)   {
    intake.set(intakeInputSpeed);
    }

  @Override
  public void setIdleMode(IdleMode elevIdleMode, IdleMode intakeIdleMode) {
    elev.setIdleMode(elevIdleMode);
    intake.setIdleMode(intakeIdleMode);
  }

  @Override
  public void setElevPosition(double height) {
    elevRelativeEncoder.setPosition(height);
  }

  // @Override
  // public void setJointPosition(double angle) {
  // offset = angle - inputs.jointPosDeg;
  // }

  @Override
  public void stop() {
    elev.stopMotor();
    intake.stopMotor();
  }
}
