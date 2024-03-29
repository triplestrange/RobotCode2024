package com.team1533.frc.robot.subsystems.rollers;

import com.revrobotics.CANSparkBase.IdleMode;
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

    public void runIntake() {
        io.runVolts(12);
    }

    public void runOutake() {
        io.runVolts(-12);
    }

    public void intakeOff() {
        io.stop();
    }

    public boolean getIntakeSensor() {
        return io.getSensor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("intake sensor", getIntakeSensor());

    }
}