package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private final FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(1), Constants.IntakeConstants.rollerGearing,
            0);

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

        inputs.intakeLinearVel = intakeSim.getAngularVelocityRPM() * Constants.IntakeConstants.rollerDiameterMeters
                * Math.PI / Constants.IntakeConstants.rollerGearing / 60;
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeMotorCurrent = intakeSim.getCurrentDrawAmps();
        inputs.intakeTempCelcius = 0.0;

        // Reset input
        intakeSim.setInputVoltage(0.0);
    }

    @Override
    public void runIntakeVolts(double intakeVolts) {
        intakeAppliedVolts = MathUtil.clamp(intakeVolts, -12.0, 12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void stop() {
        intakeAppliedVolts = 0.0;
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }
}
