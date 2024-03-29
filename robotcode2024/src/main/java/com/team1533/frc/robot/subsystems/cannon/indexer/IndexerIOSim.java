package com.team1533.frc.robot.subsystems.cannon.indexer;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.team1533.frc.robot.Constants;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim IndexerSim = new FlywheelSim(DCMotor.getNEO(1), Constants.IndexerConstants.rollerGearing,
            1);

    double IndexerMotorCurrent = 0.0;
    double IndexerAppliedVolts = 0.0;

    private boolean wasNotAuto = true;

    public IndexerIOSim() {

        IndexerSim.setState(0.0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {

        // Assume starting at ~80 degrees
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            IndexerSim.setState(0.0);
            wasNotAuto = false;
        }
        if (!DriverStation.isAutonomousEnabled()) {
            wasNotAuto = true;
        }

        IndexerSim.update(Constants.LoggerConstants.kDt);

        inputs.linearVel = IndexerSim.getAngularVelocityRPM() * Constants.IndexerConstants.rollerDiameterMeters
                * Math.PI / Constants.IndexerConstants.rollerGearing / 60;
        inputs.appliedVolts = IndexerAppliedVolts;
        inputs.motorCurrent = IndexerSim.getCurrentDrawAmps();
        inputs.tempCelcius = 0.0;

        // Reset input
        IndexerSim.setInputVoltage(0.0);
    }

    @Override
    public void runVolts(double IndexerVolts) {
        IndexerAppliedVolts = MathUtil.clamp(IndexerVolts, -12.0, 12.0);
        IndexerSim.setInputVoltage(IndexerAppliedVolts);
    }

    @Override
    public void stop() {
        IndexerAppliedVolts = 0.0;
        IndexerSim.setInputVoltage(IndexerAppliedVolts);
    }
}
