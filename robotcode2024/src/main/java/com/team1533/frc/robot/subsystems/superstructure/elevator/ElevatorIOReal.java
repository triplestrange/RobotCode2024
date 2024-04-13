package com.team1533.frc.robot.subsystems.superstructure.elevator;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorIOReal implements ElevatorIO {
  public CANSparkMax elev = new CANSparkMax(Constants.CAN.ELEVATOR, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(Constants.CAN.IPIVOT, MotorType.kBrushless);

  public SparkPIDController elevController = elev.getPIDController();

  public RelativeEncoder elevRelativeEncoder = elev.getEncoder();
  public DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(Constants.ELECTRICAL.intakeAbsInput);
  public RelativeEncoder intakeRelEncoder = intake.getEncoder();

  private double winchInput;
  private double intakeInputVolts;
  private double intakeInputSpeed;

  private double relAngle = 0;

  private double offset;

  public ElevatorIOReal() {

    intake.setInverted(true);
    elev.setInverted(false);

    elev.setIdleMode(IdleMode.kBrake);
    elev.setSmartCurrentLimit(Constants.ELECTRICAL.elevatorCurrentLimit);

    intake.setIdleMode(IdleMode.kBrake);
    intake.setSmartCurrentLimit(Constants.ELECTRICAL.intakeCurrentLimit);

    elevController.setP(ElevatorConstants.kP);
    elevController.setI(ElevatorConstants.kI);
    elevController.setD(ElevatorConstants.kD);
    elevController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);

    elevController.setSmartMotionMaxVelocity(ElevatorConstants.maxVel, 0);
    elevController.setSmartMotionMinOutputVelocity(ElevatorConstants.minVel, 0);
    elevController.setSmartMotionMaxAccel(ElevatorConstants.maxAcc, 0);
    elevController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.allowedErr, 0);

    elev.burnFlash();
    intake.burnFlash();

    elevRelativeEncoder.setPositionConversionFactor(ElevatorConstants.elevPosConv);
    elevRelativeEncoder.setVelocityConversionFactor(
        ElevatorConstants.elevDrumRadiusMeters * ElevatorConstants.elevSimPosConv / 60);
    elevRelativeEncoder.setPosition(0);

    intakeRelEncoder.setPositionConversionFactor(1.0/40.0);
    intakeRelEncoder.setPosition(relAngle);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.jointAbsoluteEncoderConnected = intakeEncoder.isConnected();

    inputs.elevMotorConnected = !elev.getFault(FaultID.kSensorFault);
    inputs.jointMotorConnected = !intake.getFault(FaultID.kSensorFault);

    inputs.elevatorPosInches = elevRelativeEncoder.getPosition();
    inputs.elevatorVelInchesPerSecond = elevRelativeEncoder.getVelocity();
    inputs.winchAppliedVolts = elev.getAppliedOutput() * elev.getBusVoltage();
    inputs.winchMotorCurrent = elev.getOutputCurrent();
    inputs.winchTempCelcius = elev.getMotorTemperature();
    inputs.winchInputVolts = winchInput;
    // if (!inputs.jointAbsoluteEncoderConnected) {
    //   inputs.jointPosDeg = MathUtil
    //       .inputModulus(-intakeRelEncoder.getPosition() * 180, -160, 20) + relAngle;
    // } else {
      inputs.jointPosDeg = MathUtil.inputModulus(
          -intakeEncoder.getAbsolutePosition() * 180 - JointConstants.intakeAbsOffset - offset, -160, 20);
      relAngle = inputs.jointPosDeg;

    // }
    inputs.jointAppliedVolts = intake.getAppliedOutput() * intake.getBusVoltage();
    inputs.jointMotorCurrent = intake.getOutputCurrent();
    inputs.jointTempCelcius = intake.getMotorTemperature();
    inputs.jointInputVolts = intakeInputVolts;
    inputs.jointInputSpeed = intakeInputSpeed;

  }

  @Override
  public void runHeightSetpoint(double height) {
    elevController.setReference(height, CANSparkMax.ControlType.kPosition);

  }

  @Override
  public void runWinchVolts(double elevVolts) {
    winchInput = elevVolts;
    elev.setVoltage(winchInput);
  }

  // @Override
  // public void runJointVolts(double intakeVolts) {
  // intakeInputVolts = intakeVolts;
  // intake.setVoltage(intakeInputVolts);
  // }
  @Override
  public void runJointPower(double jointPower) {
    intakeInputSpeed = jointPower;
    intake.set(intakeInputSpeed);
  }

  @Override
  public void setIdleMode(IdleMode elevIdleMode, IdleMode intakeIdleMode) {
    elev.setIdleMode(elevIdleMode);
    intake.setIdleMode(intakeIdleMode);
  }

  @Override
  public void setElevPosition(double height) {
    elevRelativeEncoder.setPosition(height);
  }

  // @Override
  // public void setJointPosition(double angle) {
  // offset = angle - inputs.jointPosDeg;
  // }

  @Override
  public void stop() {
    elev.stopMotor();
    intake.stopMotor();
  }
}
