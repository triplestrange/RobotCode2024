package com.team1533.frc.robot.subsystems.leds;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team1533.frc.robot.Constants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Leds extends SubsystemBase {

    public Spark leds = new Spark(Constants.ELECTRICAL.ledPWMInput);

    @RequiredArgsConstructor
    public enum LedMode {
        DEFAULT(() -> -0.93),
        AUTO(() -> -0.57),
        HAS_NOTE(() -> 0.75),
        AUTO_SHOOT(() -> 0.35),
        FLYWHEEL_CHECK(() -> 0),
        PIVOT_CHECK(() -> 0),
        HEADING_CHECK(() -> 0),
        VELOCITY_CHECK(() -> 0),
        ROTATION_CHECK(() -> 0),
        SWERVE_CHECK(() -> 0),
        CLIMBING(() -> -0.57),
        AUTO_ALIGN(() -> 0.19),
        INTAKING(() -> 0.15);

        private final DoubleSupplier ledSetpoint;

        private double getLEDSetpoint() {
            return ledSetpoint.getAsDouble();
        }
    }

    @AutoLogOutput(key = "Leds/mode")
    @Setter
    @Getter
    private LedMode mode = LedMode.DEFAULT;

    public Leds() {
        setDefaultCommand(new InstantCommand(() -> setMode(LedMode.DEFAULT)));

    }

    @Override
    public void periodic() {

        leds.set(getMode().getLEDSetpoint());

    }

}