package com.team1533.frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d rollPosition = new Rotation2d();
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};
        public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
        public double yawVelocityRadPerSec = 0.0;
        public double tempCelcius = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {
    }

    default void reset() {
    }
}