package frc.robot.subsystems.cannon.flywheel;

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

public class FlyWheelIOSim implements FlyWheelIO {
    private final FlywheelSim leftFlywheelSim = new FlywheelSim(DCMotor.getNEO(1),
            2,
            1);
    private final FlywheelSim rightFlywheelSim = new FlywheelSim(DCMotor.getNEO(1),
            2,
            1);

    double leftMotorCurrent = 0.0;
    double leftAppliedVolts = 0.0;

    private boolean wasNotAuto = true;

    public FlyWheelIOSim() {

        leftFlywheelSim.setState(0.0);
    }

    @Override
    public void updateInputs(FlyWheelIOInputs inputs) {

        // Assume starting at ~80 degrees
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            leftFlywheelSim.setState(0.0);
            wasNotAuto = false;
        }
        if (!DriverStation.isAutonomousEnabled()) {
            wasNotAuto = true;
        }

        leftFlywheelSim.update(Constants.LoggerConstants.kDt);

        inputs.leftVel = leftFlywheelSim.getAngularVelocityRPM() * 2 / 60;
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftMotorCurrent = leftFlywheelSim.getCurrentDrawAmps();
        inputs.leftTempCelcius = 0.0;

        // Reset input
        leftFlywheelSim.setInputVoltage(0.0);
    }
}
