package com.team1533.frc.robot.subsystems.cannon.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class FlyWheelIOReal implements FlyWheelIO {
  private CANSparkMax lFlyWheel;
  private CANSparkMax rFlyWheel;

  private SparkPIDController lFWController;
  private SparkPIDController rFWController;

  private RelativeEncoder lFWEncoder;
  private RelativeEncoder rFWEncoder;

  private double lFlyWheelSetpoint;
  private double rFlyWheelSetpoint;

  public FlyWheelIOReal() {
    lFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELL, MotorType.kBrushless);
    rFlyWheel = new CANSparkMax(Constants.CAN.FLYWHEELR, MotorType.kBrushless);

    lFlyWheel.restoreFactoryDefaults();
    rFlyWheel.restoreFactoryDefaults();

    lFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);
    rFlyWheel.setSmartCurrentLimit(Constants.ELECTRICAL.flyWheelCurrentLimit);

    lFWController = lFlyWheel.getPIDController();
    lFWEncoder = lFlyWheel.getEncoder();

    rFWController = rFlyWheel.getPIDController();
    rFWEncoder = rFlyWheel.getEncoder();

    int smartMotionSlot = 0;

    // fly wheel
    lFWController.setP(FlyWheelConstants.flyWheelkP);
    lFWController.setI(FlyWheelConstants.flyWheelkI);
    lFWController.setD(FlyWheelConstants.flyWheelkD);
    lFWController.setIZone(FlyWheelConstants.flyWheelkIz);
    lFWController.setFF(FlyWheelConstants.flyWheelkFF);
    lFWController.setOutputRange(FlyWheelConstants.flyWheelkMinOutput,
        FlyWheelConstants.flyWheelkMaxOutput);

    lFWController.setSmartMotionMaxVelocity(FlyWheelConstants.flyWheelmaxVel, smartMotionSlot);
    lFWController.setSmartMotionMinOutputVelocity(FlyWheelConstants.flyWheelminVel, smartMotionSlot);
    lFWController.setSmartMotionMaxAccel(FlyWheelConstants.flyWheelmaxAcc, smartMotionSlot);
    lFWController.setSmartMotionAllowedClosedLoopError(FlyWheelConstants.flyWheelallowedErr,
        smartMotionSlot);

    rFWController.setP(FlyWheelConstants.flyWheelkP);
    rFWController.setI(FlyWheelConstants.flyWheelkI);
    rFWController.setD(FlyWheelConstants.flyWheelkD);
    rFWController.setIZone(FlyWheelConstants.flyWheelkIz);
    rFWController.setFF(FlyWheelConstants.flyWheelkFF);
    rFWController.setOutputRange(FlyWheelConstants.flyWheelkMinOutput,
        FlyWheelConstants.flyWheelkMaxOutput);

    rFWController.setSmartMotionMaxVelocity(FlyWheelConstants.flyWheelmaxVel, smartMotionSlot);
    rFWController.setSmartMotionMinOutputVelocity(FlyWheelConstants.flyWheelminVel, smartMotionSlot);
    rFWController.setSmartMotionMaxAccel(FlyWheelConstants.flyWheelmaxAcc, smartMotionSlot);
    rFWController.setSmartMotionAllowedClosedLoopError(FlyWheelConstants.flyWheelallowedErr,
        smartMotionSlot);

    lFlyWheel.enableVoltageCompensation(12);
    rFlyWheel.enableVoltageCompensation(12);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    inputs.leftMotorConnected = !lFlyWheel.getFault(FaultID.kSensorFault);
    inputs.rightMotorConnected = !rFlyWheel.getFault(FaultID.kSensorFault);

    inputs.leftVel = lFWEncoder.getVelocity();
    inputs.leftInputVolts = lFlyWheel.getAppliedOutput() * lFlyWheel.getBusVoltage();
    inputs.leftMotorCurrent = lFlyWheel.getOutputCurrent();
    inputs.leftTempCelcius = lFlyWheel.getMotorTemperature();

    inputs.rightVel = rFWEncoder.getVelocity();
    inputs.rightInputVolts = rFlyWheel.getAppliedOutput() * rFlyWheel.getBusVoltage();
    inputs.rightMotorCurrent = rFlyWheel.getOutputCurrent();
    inputs.rightTempCelcius = rFlyWheel.getMotorTemperature();

    inputs.leftSetpoint = lFlyWheelSetpoint;
    inputs.rightSetpoint = rFlyWheelSetpoint;

  }

  @Override
  public void runSpeed(double RPM) {
    lFlyWheelSetpoint = RPM;
    rFlyWheelSetpoint = Math.abs(RPM) - FlyWheelConstants.rotationalSpeed / 2;

    if (lFlyWheelSetpoint == 0) {
      lFlyWheel.set(0);
    }
    if (rFlyWheelSetpoint == 0) {
      rFlyWheel.set(0);
    }
    lFWController.setReference(lFlyWheelSetpoint, ControlType.kVelocity);
    rFWController.setReference(rFlyWheelSetpoint, ControlType.kVelocity);
  }

  @Override
  public void setLeftSpeed(double RPM) {
    lFWController.setReference(RPM, ControlType.kVelocity);
  }

  @Override
  public void setRightSpeed(double RPM) {
    rFWController.setReference(RPM, ControlType.kVelocity);
  }

  @Override
  public void setIdleMode(IdleMode flyWheelIdleMode) {
    lFlyWheel.setIdleMode(flyWheelIdleMode);
    rFlyWheel.setIdleMode(flyWheelIdleMode);
  }

  @Override
  public void stop() {
    lFlyWheel.stopMotor();
    rFlyWheel.stopMotor();
  }
}
