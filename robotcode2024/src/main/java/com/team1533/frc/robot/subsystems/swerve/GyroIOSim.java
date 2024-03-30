package com.team1533.frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.team1533.frc.robot.Constants;
import com.team1533.frc.robot.subsystems.swerve.GyroIO.GyroIOInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;

public class GyroIOSim implements GyroIO {

    private double velocity = 0.0;
    private Rotation2d heading = new Rotation2d();

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
        inputs.yawPosition = heading;
        inputs.yawVelocityRadPerSec = velocity;
    }

    @Override
    public void addOffset(ChassisSpeeds speeds) {
        velocity = speeds.omegaRadiansPerSecond;
        heading = heading.plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.LoggerConstants.kDt));
    }

    @Override
    public void reset() {
    }
}
