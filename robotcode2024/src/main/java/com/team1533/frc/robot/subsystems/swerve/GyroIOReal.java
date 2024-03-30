package com.team1533.frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/** IO implementation for Pigeon2 */
public class GyroIOReal implements GyroIO {
    private final AHRS navX;
    private DoubleSupplier yaw;
    private DoubleSupplier roll;
    private DoubleSupplier pitch;

    public GyroIOReal() {
        navX = new AHRS(SPI.Port.kMXP);

        yaw = () -> navX.getFusedHeading();
        roll = () -> navX.getRoll();
        pitch = () -> navX.getPitch();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getAsDouble());
        inputs.yawVelocityRadPerSec = navX.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);

        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getAsDouble());
    }

    @Override
    public void addOffset(ChassisSpeeds speeds) {
    }

    @Override
    public void reset() {
        navX.reset();
    }
}