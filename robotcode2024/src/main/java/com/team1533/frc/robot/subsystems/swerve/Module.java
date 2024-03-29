package com.team1533.frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

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

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRads,
        inputs.turnPosition);
  }
}