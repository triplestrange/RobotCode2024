package frc.robot.subsystems.cannon.flywheel;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.cannon.indexer.IndexerIO.IndexerIOInputs;

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
    lFWController.setP(Constants.ShooterConstants.flyWheelkP);
    lFWController.setI(Constants.ShooterConstants.flyWheelkI);
    lFWController.setD(Constants.ShooterConstants.flyWheelkD);
    lFWController.setIZone(Constants.ShooterConstants.flyWheelkIz);
    lFWController.setFF(Constants.ShooterConstants.flyWheelkFF);
    lFWController.setOutputRange(Constants.ShooterConstants.flyWheelkMinOutput,
        Constants.ShooterConstants.flyWheelkMaxOutput);

    lFWController.setSmartMotionMaxVelocity(Constants.ShooterConstants.flyWheelmaxVel, smartMotionSlot);
    lFWController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.flyWheelminVel, smartMotionSlot);
    lFWController.setSmartMotionMaxAccel(Constants.ShooterConstants.flyWheelmaxAcc, smartMotionSlot);
    lFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr,
        smartMotionSlot);

    rFWController.setP(Constants.ShooterConstants.flyWheelkP);
    rFWController.setI(Constants.ShooterConstants.flyWheelkI);
    rFWController.setD(Constants.ShooterConstants.flyWheelkD);
    rFWController.setIZone(Constants.ShooterConstants.flyWheelkIz);
    rFWController.setFF(Constants.ShooterConstants.flyWheelkFF);
    rFWController.setOutputRange(Constants.ShooterConstants.flyWheelkMinOutput,
        Constants.ShooterConstants.flyWheelkMaxOutput);

    rFWController.setSmartMotionMaxVelocity(Constants.ShooterConstants.flyWheelmaxVel, smartMotionSlot);
    rFWController.setSmartMotionMinOutputVelocity(Constants.ShooterConstants.flyWheelminVel, smartMotionSlot);
    rFWController.setSmartMotionMaxAccel(Constants.ShooterConstants.flyWheelmaxAcc, smartMotionSlot);
    rFWController.setSmartMotionAllowedClosedLoopError(Constants.ShooterConstants.flyWheelallowedErr,
        smartMotionSlot);

    lFlyWheel.enableVoltageCompensation(12);
    rFlyWheel.enableVoltageCompensation(12);
  }

  @Override
  public void updateInputs(FlyWheelIOInputs inputs) {
    inputs.leftMotorConnected = lFlyWheel.getFault(FaultID.kSensorFault);
    inputs.rightMotorConnected = rFlyWheel.getFault(FaultID.kSensorFault);

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
    rFlyWheelSetpoint = Math.abs(RPM) - Constants.ShooterConstants.rotationalSpeed / 2;

    if (lFlyWheelSetpoint == 0) {
      lFlyWheel.stopMotor();
    }
    if (rFlyWheelSetpoint == 0) {
      rFlyWheel.stopMotor();
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
