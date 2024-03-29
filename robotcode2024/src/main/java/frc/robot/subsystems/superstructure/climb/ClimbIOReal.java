package frc.robot.subsystems.superstructure.climb;

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
import com.revrobotics.CANSparkBase.ControlType;
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
import frc.robot.subsystems.superstructure.climb.Climb.ClimbPosition;
import frc.robot.subsystems.superstructure.elevator.Elevator.IntakePosition;

public class ClimbIOReal implements ClimbIO {

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

  public ClimbIOReal() {

    lWinch = new CANSparkMax(Constants.CAN.CLIMBL, MotorType.kBrushless);
    rWinch = new CANSparkMax(Constants.CAN.CLIMBR, MotorType.kBrushless);

    lWinch.restoreFactoryDefaults();
    rWinch.restoreFactoryDefaults();

    lWinch.setIdleMode(IdleMode.kBrake);
    lWinch.setSmartCurrentLimit(Constants.ELECTRICAL.climbCurrentLimit);
    lWinch.setInverted(true);

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

    int smartMotionSlot = 0;
    lWinchController.setSmartMotionMaxVelocity(Constants.ElevatorConstants.maxVel, smartMotionSlot);
    lWinchController.setSmartMotionMinOutputVelocity(Constants.ElevatorConstants.minVel, smartMotionSlot);
    lWinchController.setSmartMotionMaxAccel(Constants.ElevatorConstants.maxAcc, smartMotionSlot);
    lWinchController.setSmartMotionAllowedClosedLoopError(Constants.ElevatorConstants.allowedErr, smartMotionSlot);

    rWinchController.setSmartMotionMaxVelocity(Constants.ClimbConstants.maxVel, smartMotionSlot);
    rWinchController.setSmartMotionMinOutputVelocity(Constants.ClimbConstants.minVel, smartMotionSlot);
    rWinchController.setSmartMotionMaxAccel(Constants.ClimbConstants.maxAcc, smartMotionSlot);
    rWinchController.setSmartMotionAllowedClosedLoopError(Constants.ClimbConstants.allowedErr, smartMotionSlot);

    lWinch.burnFlash();
    rWinch.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftMotorConnected = lWinch.getFault(FaultID.kSensorFault);
    inputs.rightMotorConnected = rWinch.getFault(FaultID.kSensorFault);

    inputs.leftInputVolts = lWinchPower;
    inputs.leftMotorCurrent = lWinch.getOutputCurrent();
    inputs.leftAppliedVolts = lWinch.getAppliedOutput() * lWinch.getBusVoltage();

    inputs.leftPosMeters = lWinchEncoder.getPosition();
    inputs.leftVelMetersPerSecond = lWinchEncoder.getVelocity();

    inputs.rightInputVolts = rWinchPower;
    inputs.rightMotorCurrent = rWinch.getOutputCurrent();
    inputs.rightAppliedVolts = rWinch.getAppliedOutput() * rWinch.getBusVoltage();

    inputs.rightPosMeters = rWinchEncoder.getPosition();
    inputs.rightVelMetersPerSecond = rWinchEncoder.getVelocity();

    inputs.leftTempCelcius = lWinch.getMotorTemperature();
    inputs.rightTempCelcius = rWinch.getMotorTemperature();

  }

  /** Run to setpoint IntakePosition in meters and degrees */
  @Override
  public void runClimbSetpoints(ClimbPosition position) {
    lWinchController.setReference(position.getLeftClimb(), ControlType.kPosition);
    rWinchController.setReference(position.getRightClimb(), ControlType.kPosition);
  }

  /** Run motors at volts */
  @Override
  public void runLeftVolts(double leftVolts) {
    lWinch.setVoltage(leftVolts);
  }

  @Override
  public void runLeftSpeed(double leftSpeed) {
    lWinch.set(leftSpeed);
  }

  @Override
  public void runRightSpeed(double rightSpeed) {
    rWinch.set(rightSpeed);
  }

  @Override
  public void runRightVolts(double rightVolts) {
    rWinch.setVoltage(rightVolts);
  }

  /** Set brake mode enabled */
  @Override
  public void setIdleMode(IdleMode idleMode) {
    lWinch.setIdleMode(idleMode);
    rWinch.setIdleMode(idleMode);
  }

  /** Sets position of internal encoder in inches */
  @Override
  public void setLeftClimb(double setpoint) {
    lWinchController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setRightClimb(double setpoint) {
    rWinchController.setReference(setpoint, ControlType.kPosition);
  }

  /** Stops motors */
  @Override
  public void stop() {
    lWinch.stopMotor();
    rWinch.stopMotor();
  }
}
