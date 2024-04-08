package com.team1533.frc.robot.subsystems.cannon.indexer;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOReal implements IndexerIO {
  public CANSparkMax indexer = new CANSparkMax(Constants.CAN.INDEXER, MotorType.kBrushless);

  public RelativeEncoder IndexerRelativeEncoder = indexer.getEncoder();
  private final DigitalInput indexerSensor;

  private double IndexerInput;

  private double offset;

  public IndexerIOReal() {

    indexer.setIdleMode(IdleMode.kBrake);
    indexer.setSmartCurrentLimit(Constants.ELECTRICAL.rollerCurrentLimit);

    indexer.burnFlash();

    IndexerRelativeEncoder.setVelocityConversionFactor(
        IndexerConstants.rollerDiameterMeters * Math.PI / IndexerConstants.rollerGearing / 60);

    double motorCurrent = indexer.getOutputCurrent();
    double appliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();

    double linearVEl = IndexerRelativeEncoder.getVelocity();

    double tempCelcius = indexer.getMotorTemperature();

    indexerSensor = new DigitalInput(Constants.ELECTRICAL.indexerDigitalInput);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorConnected = !indexer.getFault(FaultID.kSensorFault);

    inputs.linearVel = IndexerRelativeEncoder.getVelocity();
    inputs.inputVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.motorCurrent = indexer.getOutputCurrent();
    inputs.tempCelcius = indexer.getMotorTemperature();
    inputs.inputVolts = IndexerInput;

    inputs.sensor = !indexerSensor.get();

  }

  @Override
  public void runVolts(double IndexerVolts) {
    indexer.setVoltage(IndexerVolts);
  }

  @Override
  public void runSpeed(double speed) {
    indexer.set(speed);
  }

  @Override
  public void setIdleMode(IdleMode IndexerIdleMode) {
    indexer.setIdleMode(IndexerIdleMode);
  }

  @Override
  public void stop() {
    indexer.stopMotor();
  }
}
