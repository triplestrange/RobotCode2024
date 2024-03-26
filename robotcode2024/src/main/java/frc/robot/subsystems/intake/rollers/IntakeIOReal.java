package frc.robot.subsystems.intake.rollers;

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
        Constants.IntakeConstants.rollerDiameterMeters * Math.PI / Constants.IntakeConstants.rollerGearing / 60);

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
