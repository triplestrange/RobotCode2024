package com.team1533.frc.robot.subsystems.superstructure.arm;

import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.superstructure.arm.ArmIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim jointSim = new SingleJointedArmSim(DCMotor.getNEO(2),
            2, Units.lbsToKilograms(4.916),
            Units.inchesToMeters(11.822640),
            Units.degreesToRadians(Constants.IntakeConstants.minAngle),
            Units.degreesToRadians(Constants.IntakeConstants.maxAngle),
            true,
            0);

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;

    private boolean wasNotAuto = true;

    private double jointAppliedVolts = 0.0;
    private double jointPositionOffset = 0.0;

    public ArmIOSim() {
        jointSim.setState(0.0, 0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            controllerNeedsReset = true;
        }
        // Assume starting at ~80 degrees
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            jointSim.setState(0.0, 0.0);
            wasNotAuto = false;
        }
        if (!DriverStation.isAutonomousEnabled()) {
            wasNotAuto = true;
        }

        jointSim.update(Constants.LoggerConstants.kDt);

        inputs.posDeg = jointSim.getAngleRads() + jointPositionOffset;
        inputs.jointVelDegPerSecond = jointSim.getVelocityRadPerSec();
        inputs.leftMotorCurrent = jointSim.getCurrentDrawAmps();

        // Reset input
        jointSim.setInputVoltage(0.0);
    }

    @Override
    public void runVolts(double volts) {
        jointAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        jointSim.setInputVoltage(jointAppliedVolts);
    }

    @Override
    public void stop() {

        jointAppliedVolts = 0.0;
        jointSim.setInputVoltage(jointAppliedVolts);
    }
}
