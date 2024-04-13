package com.team1533.frc.robot.subsystems.cannon.flywheel;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlyWheel extends SubsystemBase {
    private static FlyWheel instance;

    private FlyWheelIO io;
    private FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

    public static FlyWheel getInstance() {
        return instance;
    }

    public static FlyWheel initialize(FlyWheelIO io) {
        if (instance == null) {
            instance = new FlyWheel(io);
        }
        return instance;
    }

    public enum Mode {
        TELEOP,
        AUTO
    }

    @Getter
    @AutoLogOutput(key = "Flywheels/mode")
    private Mode mode = Mode.TELEOP;

    /**
     * Creates a new Shooter.
     */

    public FlyWheel(FlyWheelIO flyWheelIO) {
        super();
        io = flyWheelIO;
        io.updateInputs(inputs);

        io.setIdleMode(IdleMode.kCoast);
    }

    public double getLeftSpeed() {
        return inputs.leftVel;
    }

    public double getRightSpeed() {
        return inputs.rightVel;
    }

    public void setFWSpeed(double RPM) {
        io.runSpeed(RPM);
    }

    public void setLeftSpeed(double RPM) {
        io.setLeftSpeed(RPM);
    }

    public void setRightSpeed(double RPM) {
        io.setRightSpeed(RPM);
    }

    public void flyWheelOn() {
        io.setLeftSpeed(-900);
        io.setRightSpeed(900);
    }

    public void cleaningMode()  {
        io.setLeftSpeed(100);
        io.setRightSpeed(-100);
    }

    public void flyWheelOff() {
        io.setLeftSpeed(0);
        io.setRightSpeed(0);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
                Logger.processInputs("Flywheel", inputs);


        // switch (mode) {
        //     case TELEOP:
        //         setDefaultCommand(new InstantCommand(() -> setLeftSpeed(900), getInstance()));
        //         setDefaultCommand(new InstantCommand(() -> setRightSpeed(900), getInstance()));
        //         break;

        //     case AUTO:
        //         setDefaultCommand(new InstantCommand(() -> setLeftSpeed(4700), getInstance()));
        //         setDefaultCommand(new InstantCommand(() -> setRightSpeed(3000), getInstance()));
        //         break;
        // }

    }

    public void updateSmartDashBoard() {

        SmartDashboard.putNumber("left rpm", inputs.leftVel);
        SmartDashboard.putNumber("right rpm", inputs.rightVel);
        SmartDashboard.putNumber("left setpoint", inputs.leftSetpoint);
        SmartDashboard.putNumber("right setpoint", inputs.rightSetpoint);

    }

}
