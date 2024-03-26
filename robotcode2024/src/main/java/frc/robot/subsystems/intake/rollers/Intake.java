package frc.robot.subsystems.intake.rollers;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Intake instance;

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static Intake getInstance() {
        return instance;
    }

    public static Intake initialize(IntakeIO io) {
        if (instance == null) {
            instance = new Intake(io);
        }
        return instance;
    }

    /** Creates a new Intake. */
    public Intake(IntakeIO intakeIO) {
        super();

        io = intakeIO;
        io.updateInputs(inputs);

        io.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("intake sensor", io.getIntakeSensor());

    }
}