package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(1),
            Constants.ElevatorConstants.elevSimPosConv,
            Constants.ElevatorConstants.elevCarraigeKG, Constants.ElevatorConstants.elevDrumRadiusMeters,
            Units.inchesToMeters(Constants.ElevatorConstants.minHeightInches),
            Constants.ElevatorConstants.maxHeightInches, true, Constants.ElevatorConstants.minHeightInches);
    private final SingleJointedArmSim jointSim = new SingleJointedArmSim(DCMotor.getNeo550(1),
            2, Units.lbsToKilograms(4.916),
            Units.inchesToMeters(11.822640),
            Units.degreesToRadians(Constants.IntakeConstants.minAngle),
            Units.degreesToRadians(Constants.IntakeConstants.maxAngle),
            true,
            0);

    private final PIDController controller;
    double winchMotorCurrent = 0.0;
    double winchAppliedVolts = 0.0;

    double jointMotorCurrent = 0.0;
    double jointAppliedVolts = 0.0;

    private double jointPositionOffset = 0.0;
    private double elevPositionOffset = 0.0;

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;

    private boolean wasNotAuto = true;

    public ElevatorIOSim() {

        controller = new PIDController(0.0, 0.0, 0.0);
        elevatorSim.setState(0.0, 0.0);
        setElevPosition(0.0);
        jointSim.setState(0.0, 0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            controllerNeedsReset = true;
        }
        // Assume starting at ~80 degrees
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            elevatorSim.setState(0.0, 0.0);
            jointSim.setState(0.0, 0.0);
            wasNotAuto = false;
        }
        if (!DriverStation.isAutonomousEnabled()) {
            wasNotAuto = true;
        }

        elevatorSim.update(Constants.LoggerConstants.kDt);
        jointSim.update(Constants.LoggerConstants.kDt);

        inputs.elevatorPosInches = elevatorSim.getPositionMeters();
        inputs.elevatorVelInchesPerSecond = elevatorSim.getVelocityMetersPerSecond();
        inputs.winchAppliedVolts = winchAppliedVolts;
        inputs.winchMotorCurrent = elevatorSim.getCurrentDrawAmps();
        inputs.winchTempCelcius = 0.0;

        // Reset input
        elevatorSim.setInputVoltage(0.0);

        inputs.jointPosDeg = jointSim.getAngleRads() + jointPositionOffset;
        inputs.joinVelDegPerSecond = jointSim.getVelocityRadPerSec();
        inputs.jointAppliedVolts = jointAppliedVolts;
        inputs.jointMotorCurrent = jointSim.getCurrentDrawAmps();
        inputs.jointTempCelcius = 0.0;

        // Reset input
        jointSim.setInputVoltage(0.0);
    }

    @Override
    public void runHeightSetpoint(double height) {
        if (!closedLoop) {
            controllerNeedsReset = true;
            closedLoop = true;
        }
        if (controllerNeedsReset) {
            controller.reset();
            controllerNeedsReset = false;
        }
        runWinchVolts(controller.calculate(elevatorSim.getPositionMeters(), height));
    }

    @Override
    public void runWinchVolts(double elevVolts) {
        closedLoop = false;
        winchAppliedVolts = MathUtil.clamp(elevVolts, -12.0, 12.0);
        elevatorSim.setInputVoltage(winchAppliedVolts);
    }

    @Override
    public void runJointVolts(double intakeVolts) {
        jointAppliedVolts = MathUtil.clamp(intakeVolts, -12.0, 12.0);
        jointSim.setInputVoltage(jointAppliedVolts);
    }

    @Override
    public void setElevPosition(double height) {
        this.elevPositionOffset = height - elevatorSim.getPositionMeters();
    }

    @Override
    public void setJointPosition(double angle) {
        jointPositionOffset = angle - jointSim.getAngleRads();

    }

    @Override
    public void stop() {
        winchAppliedVolts = 0.0;
        elevatorSim.setInputVoltage(winchAppliedVolts);
        jointAppliedVolts = 0.0;
        jointSim.setInputVoltage(jointAppliedVolts);
    }
}
