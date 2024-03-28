package frc.robot.subsystems.intake.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        intakeController = new ProfiledPIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI,
                Constants.IntakeConstants.kD,
                new Constraints(Constants.IntakeConstants.kMaxAngularSpeedMetersPerSecond,
                        Constants.IntakeConstants.kMaxAngularAccelerationMetersPerSecondSquared));

        elevSetpoint = inputs.elevatorPosMeters;

    }

    public static class IntakePosition {
        private double elevPos;
        private double intakeAng;

        public IntakePosition(double elevPos, double intakeAng) {
            this.elevPos = elevPos;
            this.intakeAng = intakeAng;
        }

        // height from bottom of carriage
        public double getHeight() {
            return elevPos;
        }

        public double getAngle() {
            return intakeAng;
        }
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
        return new IntakePosition(inputs.elevatorPosMeters, inputs.jointPosDeg);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
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
