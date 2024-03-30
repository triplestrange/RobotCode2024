package com.team1533.frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.team1533.frc.robot.util.Alert;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {

  private static final String[] moduleNames = new String[] { "FL", "FR", "BL", "BR" };

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  // Alerts
  private final Alert driveMotorDisconnected;
  private final Alert turnMotorDisconnected;

  public Module(ModuleIO moduleIO, int index) {
    this.io = moduleIO;
    this.index = index;

    driveMotorDisconnected = new Alert(moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
    turnMotorDisconnected = new Alert(moduleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);

  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Display alerts
    driveMotorDisconnected.set(!inputs.driveMotorConnected);
    turnMotorDisconnected.set(!inputs.turnMotorConnected);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityRadsPerSec
            + inputs.turnVelocityPerSec.getRadians() * ModuleConstants.kWheelDiameterMeters / 2,
        inputs.turnPosition);
  }

  /**
   * Zeros all the SwerveModule encoders.
   */

  public void resetEncoders() {
    io.resetEncoders();
  }

  /**
   * Physically zeroes wheel. (i hope)
   */
  public void resetWheel() {
    io.runDriveVelocitySetpoint(0, 0);
    io.runTurnPositionSetpoint(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    io.setDesiredState(state);
  }

  public void setDesiredState(SwerveModuleState state, boolean forceAngle) {
    io.setDesiredState(state, forceAngle);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRads,
        inputs.turnPosition);
  }
}