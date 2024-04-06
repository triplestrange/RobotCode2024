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
        DEFAULT(() -> -0.93), // Rainbow Lava Palatte
        AUTO(() -> -0.57), // Fire Large
        AUTO_ALIGN(() -> 0),
        HAS_NOTE(() -> 0.75), // Dark Green
        AUTO_SHOOT(() -> 0.35), // Strobe Color 2
        FLYWHEEL_CHECK(() -> 0.91), // Violet
        PIVOT_CHECK(() -> 0.57), // Hot Pink
        HEADING_CHECK(() -> 0.59), // Solid Dark Red
        VELOCITY_CHECK(() -> 0.79), // Solid Blue - Green
        ROTATION_CHECK(() -> 0.81), // Solid Aqua
        SWERVE_CHECK(() -> 0.65), // Solid Orange
        CLIMBING(() -> -0.57), // Fire Large
        INTAKING(() -> 0.15); // Strobe Color 1

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

    }

    @Override
    public void periodic() {

        leds.set(getMode().getLEDSetpoint());

    }

}