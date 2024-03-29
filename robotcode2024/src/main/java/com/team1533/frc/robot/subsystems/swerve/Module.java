package com.team1533.frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.util.AbsoluteEncoder;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO moduleIO, int index) {
    this.io = moduleIO;
    this.index = index;

  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getVelocity().getValueAsDouble()
            + m_turningEncoder.getVelocity() * ModuleConstants.kWheelDiameterMeters / 2,
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    m_driveMotor.setPosition(0);
    m_turningEncoder.setPosition(m_absoluteEncoder.getAngle());
  }

  /**
   * Physically zeroes wheel. (i hope)
   */
  public void resetWheel() {
    output.Output = 0;
    m_driveMotor.setControl(output);
    m_pidController.setReference(0, ControlType.kPosition);

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }
}