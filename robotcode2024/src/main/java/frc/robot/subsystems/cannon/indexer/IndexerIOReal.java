package frc.robot.subsystems.cannon.indexer;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

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
        Constants.IndexerConstants.rollerDiameterMeters * Math.PI / Constants.IndexerConstants.rollerGearing / 60);

    double motorCurrent = Indexer.getOutputCurrent();
    double appliedVolts = Indexer.getAppliedOutput() * Indexer.getBusVoltage();

    double linearVEl = IndexerRelativeEncoder.getVelocity();

    double tempCelcius = Indexer.getMotorTemperature();

    IndexerSensor = new DigitalInput(Constants.ELECTRICAL.indexerDigitalInput);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.motorConnected = Indexer.getFault(FaultID.kSensorFault);

    inputs.linearVel = IndexerRelativeEncoder.getVelocity();
    inputs.inputVolts = Indexer.getAppliedOutput() * Indexer.getBusVoltage();
    inputs.motorCurrent = Indexer.getOutputCurrent();
    inputs.tempCelcius = Indexer.getMotorTemperature();
    inputs.inputVolts = IndexerInput;

    /*
     * inputs.joinVelDegPerSecond =
     */

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
    return !IndexerSensor.get();
  }

  @Override
  public void stop() {
    Indexer.stopMotor();
  }
}
