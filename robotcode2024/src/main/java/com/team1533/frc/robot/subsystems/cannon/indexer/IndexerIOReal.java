package com.team1533.frc.robot.subsystems.cannon.indexer;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOReal implements IndexerIO {
  public CANSparkMax Indexer = new CANSparkMax(Constants.CAN.INDEXER, MotorType.kBrushless);

  public RelativeEncoder IndexerRelativeEncoder = Indexer.getEncoder();
  private final DigitalInput IndexerSensor;

  private double IndexerInput;

  private double offset;

  public IndexerIOReal() {

    Indexer.setIdleMode(IdleMode.kBrake);
    Indexer.setSmartCurrentLimit(Constants.ELECTRICAL.rollerCurrentLimit);

    Indexer.burnFlash();

    IndexerRelativeEncoder.setVelocityConversionFactor(
        IndexerConstants.rollerDiameterMeters * Math.PI / IndexerConstants.rollerGearing / 60);

    double motorCurrent = Indexer.getOutputCurrent();
    double appliedVolts = Indexer.getAppliedOutput() * Indexer.getBusVoltage();

    double linearVEl = IndexerRelativeEncoder.getVelocity();

    double tempCelcius = Indexer.getMotorTemperature();

    IndexerSensor = new DigitalInput(Constants.ELECTRICAL.indexerDigitalInput);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorConnected = !Indexer.getFault(FaultID.kSensorFault);

    inputs.linearVel = IndexerRelativeEncoder.getVelocity();
    inputs.inputVolts = Indexer.getAppliedOutput() * Indexer.getBusVoltage();
    inputs.motorCurrent = Indexer.getOutputCurrent();
    inputs.tempCelcius = Indexer.getMotorTemperature();
    inputs.inputVolts = IndexerInput;

  }

  @Override
  public void runVolts(double IndexerVolts) {
    Indexer.setVoltage(IndexerVolts);
  }

  @Override
  public void runSpeed(double speed) {
    Indexer.set(speed);
  }

  @Override
  public void setIdleMode(IdleMode IndexerIdleMode) {
    Indexer.setIdleMode(IndexerIdleMode);
  }

  @Override
  public boolean getSensor() {
    return IndexerSensor.get();
  }

  @Override
  public void stop() {
    Indexer.stopMotor();
  }
}
