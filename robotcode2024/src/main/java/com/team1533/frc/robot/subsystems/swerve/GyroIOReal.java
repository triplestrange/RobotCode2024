package com.team1533.frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/** IO implementation for Pigeon2 */
public class GyroIOReal implements GyroIO {
    private final AHRS navX;
    private DoubleSupplier yaw;
    private DoubleSupplier roll;
    private DoubleSupplier pitch;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> pitchPositionQueue;
    private final Queue<Double> rollPositionQueue;

    public GyroIOReal() {
        navX = new AHRS(SPI.Port.kMXP);

        yaw = () -> navX.getFusedHeading();
        roll = () -> navX.getRoll();
        pitch = () -> navX.getPitch();

        yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(yaw);
        pitchPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(pitch);
        rollPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(roll);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(navX.getFusedHeading());
        inputs.yawVelocityRadPerSec = navX.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);

        inputs.pitchPosition = Rotation2d.fromDegrees(navX.getPitch());
        inputs.rollPosition = Rotation2d.fromDegrees(navX.getRoll());

        inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);
        inputs.odometryPitchPositions = pitchPositionQueue.stream().map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);
        inputs.odometryRollPositions = rollPositionQueue.stream().map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);

        yawPositionQueue.clear();
        pitchPositionQueue.clear();
        rollPositionQueue.clear();
    }

    @Override
    public void reset() {
        navX.reset();
    }
}