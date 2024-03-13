package frc.robot.subsystems.intake.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    @AutoLogOutput
    public Mechanism2d intakeMech;

    public MechanismRoot2d elevatorRoot2d;

    public MechanismLigament2d elevator;
    public MechanismLigament2d intakeStatic;
    public MechanismLigament2d intakeJoint;

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

    }

    public static class IntakePosition {
        private double elevPos;
        private double intakeAng;

        public IntakePosition(double elevPos, double intakeAng) {
            this.elevPos = elevPos;
            this.intakeAng = intakeAng;
        }

        // height from bottom of carriage
        public double getIntakeHeight() {
            return elevPos;
        }

        public double getIntakeAngle() {
            return intakeAng;
        }
    }

    @Override
    public void periodic() {
        if (elevPIDEnabled) {
            elevController.setReference(elevSetpoint, CANSparkMax.ControlType.kPosition);
        }

        if (intakePIDEnabled) {
            intakePower = intakeController.calculate(getIntakePos(), intakeSetpoint);
        }
        if (!intakeEncoder.isConnected()) {
            intakePower = 0;
        }

        intake.set(intakePower);

        intakeJoint.setAngle(getIntakePos());
        elevator.setLength(getElevPos());

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("degree", getIntakePos());
        SmartDashboard.putBoolean("Is Encoder Plugged", intakeEncoder.isConnected());
        SmartDashboard.putNumber("angle setpoint", intakeSetpoint);
        SmartDashboard.putNumber("Power", intakePower);
        SmartDashboard.putNumber("height", getElevPos());
        SmartDashboard.putNumber("height setpoint", elevSetpoint);
        SmartDashboard.putNumber("Elev Power", elevPower);

    }
}
