package com.team1533.frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.team1533.frc.robot.subsystems.superstructure.elevator.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private static Elevator m_Elevator;

    private static Intake instance;

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static Intake getInstance() {
        return instance;
    }

    public static Intake initialize(IntakeIO io) {
        if (instance == null) {
            instance = new Intake(io, m_Elevator);
        }
        return instance;
    }

    /** Creates a new Intake. */
    public Intake(IntakeIO intakeIO, Elevator m_Elevator) {
        super();

        io = intakeIO;
        io.updateInputs(inputs);

        io.setIdleMode(IdleMode.kBrake);

        this.m_Elevator = m_Elevator;
    }

    public void runIntake() {
        io.runVolts(12);
    }

    public void runOutake() {
        if (m_Elevator.getIntakePos().getAngle() < -50) {
            io.runVolts(-12);
        } else {
            io.runVolts(-4);
        }
    }

    public void intakeOff() {
        io.stop();
    }

    public boolean getIntakeSensor() {
        return inputs.sensor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }

    public void updateSmartDashBoard() {
        SmartDashboard.putBoolean("intake sensor", getIntakeSensor());

    }
}