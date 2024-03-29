package com.team1533.frc.robot.subsystems.superstructure.elevator;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team1533.lib.elevator.IntakePosition;
import com.team254.lib.util.Util;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Elevator {

    @AutoLogOutput
    public Mechanism2d intakeMech;

    public MechanismRoot2d elevatorRoot2d;

    public MechanismLigament2d elevator;
    public MechanismLigament2d intakeStatic;
    public MechanismLigament2d intakeJoint;

    private ProfiledPIDController intakeController;

    private boolean intakePIDEnabled;
    private boolean elevPIDEnabled;

    public double intakeSetpoint;

    public double elevSetpoint;

    public double elevPower;
    public double intakePower;

    private BooleanSupplier disableSupplier = DriverStation::isDisabled;

    private static Elevator instance;

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public static Elevator getInstance() {
        return instance;
    }

    public static Elevator initialize(ElevatorIO io) {
        if (instance == null) {
            instance = new Elevator(io);
        }
        return instance;
    }

    @RequiredArgsConstructor
    public enum Goal {
        STOP(new IntakePosition(0, 0)),
        AMP(new IntakePosition(34.649467, -136)),
        STOW(new IntakePosition(0, 0)),
        GROUND(new IntakePosition(0, -132)),
        TRAP(new IntakePosition(33, -136)),
        FEEDER(new IntakePosition(10, -60)),
        UNTRAP(new IntakePosition(0, 0)),
        MANUAL(new IntakePosition(0, 0));

        private final IntakePosition elevatorSetpoint;

        private IntakePosition getPos() {
            return elevatorSetpoint;
        }
    }

    @AutoLogOutput
    @Getter
    @Setter
    Goal goal = Goal.STOW;

    /**
     * Creates a new Elevator.
     */
    public Elevator(ElevatorIO elevatorIO) {
        super();

        io = elevatorIO;
        io.updateInputs(inputs);

        intakeMech = new Mechanism2d(Units.inchesToMeters(24), Units.inchesToMeters(23.625));

        elevatorRoot2d = intakeMech.getRoot("elevator root", 13, 9.5);

        elevator = elevatorRoot2d
                .append(new MechanismLigament2d("elevator", Units.inchesToMeters(23.635), 90));
        intakeStatic = elevator.append(
                new MechanismLigament2d("intake static", Units.inchesToMeters(878500), 0));
        intakeJoint = intakeStatic.append(
                new MechanismLigament2d("intake joint", Units.inchesToMeters(12.227660), 90));

        intakeController = new ProfiledPIDController(JointConstants.kP,
                JointConstants.kI,
                JointConstants.kD,
                new Constraints(JointConstants.kMaxAngularSpeedMetersPerSecond,
                        JointConstants.kMaxAngularAccelerationMetersPerSecondSquared));

        elevSetpoint = inputs.elevatorPosInches;

    }

    public void resetPIDs() {
        intakeSetpoint = getIntakePos().getAngle();
        elevSetpoint = getIntakePos().getHeight();
        intakeController.reset(intakeSetpoint);
        io.runHeightSetpoint(elevSetpoint);
        intakePIDEnabled = true;
        elevPIDEnabled = true;

    }

    public void moveElev(double motorElevPower, double motorIntakePower) {
        if (Math.abs(motorElevPower) < 0.1) {
            elevPIDEnabled = true;
        } else {
            elevPower = motorElevPower;
            io.runWinchVolts(elevPower * 12);
            elevSetpoint = getIntakePos().getHeight();
            elevPIDEnabled = false;
        }
        if (Math.abs(motorIntakePower) < 0.05) {
            intakePIDEnabled = true;
        } else {
            intakePower = motorIntakePower;
            io.runJointPower(intakePower);
            intakeSetpoint = getIntakePos().getAngle();
            intakeController.reset(intakeSetpoint);
            intakePIDEnabled = false;
        }
    }

    public void setElev(IntakePosition position) {
        elevSetpoint = position.getHeight();
        intakeSetpoint = position.getAngle();

        elevPIDEnabled = true;
        intakePIDEnabled = true;
    }

    public IntakePosition getIntakePos() {
        return new IntakePosition(inputs.elevatorPosInches, inputs.jointPosDeg);
    }

    @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
    public boolean atGoal() {
        return intakeController.atGoal() && Util.epsilonEquals(elevSetpoint, inputs.elevatorPosInches, 0.1);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (goal != Goal.MANUAL) {
            elevSetpoint = goal.getPos().getHeight();
            intakeSetpoint = goal.getPos().getAngle();
        }
        if (disableSupplier.getAsBoolean() || goal == goal.STOP) {
            io.stop();
        }
        if (elevPIDEnabled) {
            io.runHeightSetpoint(elevSetpoint);
        }

        if (intakePIDEnabled) {
            intakePower = intakeController.calculate(getIntakePos().getAngle(), intakeSetpoint);
        }
        if (!inputs.jointAbsoluteEncoderConnected) {
            intakePower = 0;
        }

        io.runJointPower(intakePower);

        intakeJoint.setAngle(getIntakePos().getAngle());
        elevator.setLength(getIntakePos().getHeight());

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("degree", getIntakePos().getAngle());
        SmartDashboard.putBoolean("Is Encoder Plugged", inputs.jointAbsoluteEncoderConnected);
        SmartDashboard.putNumber("angle setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Power", intakePower);
        SmartDashboard.putNumber("height", getIntakePos().getHeight());
        SmartDashboard.putNumber("height setpoint", elevSetpoint);
        SmartDashboard.putNumber("Elev Power", elevPower);

    }
}
