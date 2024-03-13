package frc.robot.subsystems.intake.elevator;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(1), Constants.ElevatorConstants.elevSimPosConv,
            Constants.ElevatorConstants.elevCarraigeKG, Constants.ElevatorConstants.elevDrumRadiusMeters,
            Units.inchesToMeters(Constants.ElevatorConstants.minHeightInches),
            Constants.ElevatorConstants.maxHeightInches, true, Constants.ElevatorConstants.minHeightInches);

    private double winchAppliedVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorSim.update(Constants.ElevatorConstants.kDt);
        inputs.elevatorPosMeters = elevatorSim.getPositionMeters();
        inputs.elevatorVelMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
        inputs.winchAppliedVolts = winchAppliedVolts;
    }

    @Override
    public void moveElevator(double speed) {
        elevatorSim.setInputVoltage(winchAppliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        winchAppliedVolts = volts;
        Logger.recordOutput("Setting Output", volts);
    }
}
