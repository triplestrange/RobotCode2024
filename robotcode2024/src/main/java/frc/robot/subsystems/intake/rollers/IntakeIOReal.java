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

    double intakeMotorCurrent = intake.getOutputCurrent();
    double intakeAppliedVolts = intake.getAppliedOutput() * intake.getBusVoltage();

    double intakeatorVelMetersPerSecond = intakeRelativeEncoder.getVelocity();

    double intakeTempCelcius = intake.getMotorTemperature();

    intakeSensor = new DigitalInput(Constants.ELECTRICAL.intakeDigitalInput);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected = intake.getFault(FaultID.kSensorFault);

    inputs.intakeLinearVel = intakeRelativeEncoder.getVelocity();
    inputs.intakeInputVolts = intake.getAppliedOutput() * intake.getBusVoltage();
    inputs.intakeMotorCurrent = intake.getOutputCurrent();
    inputs.intakeTempCelcius = intake.getMotorTemperature();
    inputs.intakeInputVolts = intakeInput;

    /*
     * inputs.joinVelDegPerSecond =
     */

  }

  @Override
  public void runIntakeVolts(double intakeVolts) {
    intake.setVoltage(intakeVolts);
  }

  @Override
  public void setIdleMode(IdleMode intakeIdleMode) {
    intake.setIdleMode(intakeIdleMode);
  }

  @Override
  public boolean getIntakeSensor() {
    return !intakeSensor.get();
  }

  @Override
  public void stop() {
    intake.stopMotor();
  }
}
