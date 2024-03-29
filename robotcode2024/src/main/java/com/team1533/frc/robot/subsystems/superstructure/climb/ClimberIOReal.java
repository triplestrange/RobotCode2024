package com.team1533.frc.robot.subsystems.superstructure.climb;

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
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.superstructure.climb.Climber.ClimbPosition;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberIOReal implements ClimberIO {

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

  public ClimberIOReal() {

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
    lWinchEncoder.setPositionConversionFactor(ClimberConstants.climbPosConv);

    rWinchController = rWinch.getPIDController();
    rWinchEncoder = rWinch.getEncoder();
    rWinchEncoder.setPositionConversionFactor(ClimberConstants.climbPosConv);

    // set PID coefficients
    lWinchController.setP(ClimberConstants.kP);
    lWinchController.setI(ClimberConstants.kI);
    lWinchController.setD(ClimberConstants.kD);
    lWinchController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    rWinchController.setP(ClimberConstants.kP);
    rWinchController.setI(ClimberConstants.kI);
    rWinchController.setD(ClimberConstants.kD);
    rWinchController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    int smartMotionSlot = 0;
    lWinchController.setSmartMotionMaxVelocity(ClimberConstants.maxVel, smartMotionSlot);
    lWinchController.setSmartMotionMinOutputVelocity(ClimberConstants.minVel, smartMotionSlot);
    lWinchController.setSmartMotionMaxAccel(ClimberConstants.maxAcc, smartMotionSlot);
    lWinchController.setSmartMotionAllowedClosedLoopError(ClimberConstants.allowedErr, smartMotionSlot);

    rWinchController.setSmartMotionMaxVelocity(ClimberConstants.maxVel, smartMotionSlot);
    rWinchController.setSmartMotionMinOutputVelocity(ClimberConstants.minVel, smartMotionSlot);
    rWinchController.setSmartMotionMaxAccel(ClimberConstants.maxAcc, smartMotionSlot);
    rWinchController.setSmartMotionAllowedClosedLoopError(ClimberConstants.allowedErr, smartMotionSlot);

    lWinch.burnFlash();
    rWinch.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
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
