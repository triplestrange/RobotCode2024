package com.team1533.frc.robot.subsystems.rollers;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
  public CANSparkMax intake = new CANSparkMax(Constants.CAN.ROLLERS, MotorType.kBrushless);

  public RelativeEncoder intakeRelativeEncoder = intake.getEncoder();
  private final DigitalInput intakeSensor;

  private double intakeInput;

  private double offset;

  public IntakeIOReal() {

    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(Constants.ELECTRICAL.rollerCurrentLimit);

    intake.burnFlash();

    intakeRelativeEncoder.setVelocityConversionFactor(
        IntakeConstants.rollerDiameterMeters * Math.PI / IntakeConstants.rollerGearing / 60);

    double motorCurrent = intake.getOutputCurrent();
    double appliedVolts = intake.getAppliedOutput() * intake.getBusVoltage();

    double linearVEl = intakeRelativeEncoder.getVelocity();

    double tempCelcius = intake.getMotorTemperature();

    intakeSensor = new DigitalInput(Constants.ELECTRICAL.intakeDigitalInput);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.motorConnected = intake.getFault(FaultID.kSensorFault);

    inputs.linearVel = intakeRelativeEncoder.getVelocity();
    inputs.inputVolts = intake.getAppliedOutput() * intake.getBusVoltage();
    inputs.motorCurrent = intake.getOutputCurrent();
    inputs.tempCelcius = intake.getMotorTemperature();
    inputs.inputVolts = intakeInput;

    /*
     * inputs.joinVelDegPerSecond =
     */

  }

  @Override
  public void runVolts(double intakeVolts) {
    intake.setVoltage(intakeVolts);
  }

  @Override
  public void setIdleMode(IdleMode intakeIdleMode) {
    intake.setIdleMode(intakeIdleMode);
  }

  @Override
  public boolean getSensor() {
    return !intakeSensor.get();
  }

  @Override
  public void stop() {
    intake.stopMotor();
  }
}
