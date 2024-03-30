package com.team1533.frc.robot.subsystems.rollers;

import com.team1533.frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(1), IntakeConstants.rollerGearing,
            IntakeConstants.jKGMetersPerSecondSquared);

    double intakeMotorCurrent = 0.0;
    double intakeAppliedVolts = 0.0;

    private boolean wasNotAuto = true;

    public IntakeIOSim() {

        intakeSim.setState(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        // Assume starting at ~80 degrees
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            intakeSim.setState(0.0);
            wasNotAuto = false;
        }
        if (!DriverStation.isAutonomousEnabled()) {
            wasNotAuto = true;
        }

        intakeSim.update(Constants.LoggerConstants.kDt);

        inputs.linearVel = intakeSim.getAngularVelocityRPM() * IntakeConstants.rollerDiameterMeters
                * Math.PI / IntakeConstants.rollerGearing / 60;
        inputs.appliedVolts = intakeAppliedVolts;
        inputs.motorCurrent = intakeSim.getCurrentDrawAmps();
        inputs.tempCelcius = 0.0;

        // Reset input
        intakeSim.setInputVoltage(0.0);
    }

    @Override
    public void runVolts(double intakeVolts) {
        intakeAppliedVolts = MathUtil.clamp(intakeVolts, -12.0, 12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void stop() {
        intakeAppliedVolts = 0.0;
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }
}
