package frc.robot.subsystems.superstructure.arm;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.arm.ArmIO;

public class ArmIOReal implements ArmIO {
  private CANSparkMax lPivot;
  private CANSparkMax rPivot;

  private DutyCycleEncoder pivotEncoder;

  private double inputVolts;
  private double inputSpeed;

  public ArmIOReal() {

    lPivot = new CANSparkMax(Constants.CAN.PIVOTL, MotorType.kBrushless);
    rPivot = new CANSparkMax(Constants.CAN.PIVOTR, MotorType.kBrushless);

    lPivot.restoreFactoryDefaults();
    rPivot.restoreFactoryDefaults();

    lPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);
    rPivot.setSmartCurrentLimit(Constants.ELECTRICAL.shooterPivotCurrentLimit);

    pivotEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.pivotAbsInput);

    pivotEncoder.setPositionOffset(Constants.ShooterConstants.pivotAbsOffset);

    lPivot.setInverted(true);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.leftMotorConnected = lPivot.getFault(FaultID.kSensorFault);
    inputs.absoluteEncoderConnected = pivotEncoder.isConnected();

    inputs.leftInputVolts = inputVolts;
    inputs.leftInputSpeed = inputSpeed;
    inputs.leftMotorCurrent = lPivot.getOutputCurrent();
    inputs.leftAppliedVolts = lPivot.getAppliedOutput() * lPivot.getBusVoltage();

    inputs.posDeg = MathUtil.inputModulus(
        -pivotEncoder.getAbsolutePosition() * 360 - Constants.ShooterConstants.pivotAbsOffset, 30, -330);

    inputs.leftTempCelcius = lPivot.getMotorTemperature();

    inputs.rightMotorConnected = rPivot.getFault(FaultID.kSensorFault);

    inputs.rightInputVolts = inputVolts;
    inputs.rightInputSpeed = inputSpeed;
    inputs.rightMotorCurrent = rPivot.getOutputCurrent();
    inputs.rightAppliedVolts = rPivot.getAppliedOutput() * rPivot.getBusVoltage();

    inputs.rightTempCelcius = rPivot.getMotorTemperature();

  }

  @Override
  public void runVolts(double volts) {
    inputVolts = volts;
    lPivot.setVoltage(inputVolts);
    rPivot.setVoltage(inputVolts);
  }

  @Override
  public void runPower(double power) {
    inputSpeed = power;
    lPivot.set(power);
    rPivot.set(power);
  }

  @Override
  public void setIdleMode(IdleMode leftIdleMode, IdleMode rightIdleMode) {
    lPivot.setIdleMode(leftIdleMode);
    rPivot.setIdleMode(rightIdleMode);
  }

  @Override
  public void stop() {
    lPivot.stopMotor();
    rPivot.stopMotor();
  }
}
