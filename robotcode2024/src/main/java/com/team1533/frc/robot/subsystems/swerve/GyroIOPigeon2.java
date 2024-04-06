package com.team1533.frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.security.PrivilegedAction;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private static final int id = 0;

    private final Pigeon2 pigeon;
    private final StatusSignal<Double> yaw;
    private final StatusSignal<Double> yawVelocity;
    private final double totalDistanceYaw;
    private final StatusSignal<Double> pitchPosition;
    private final StatusSignal<Double> rollPosition;
    private final StatusSignal<Double> tempCelcius;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(id, "*");
        yaw = pigeon.getYaw();

        yawVelocity = pigeon.getAngularVelocityZWorld();

        totalDistanceYaw = pigeon.getAngle();

        pitchPosition = pigeon.getPitch();

        rollPosition = pigeon.getRoll();

        tempCelcius = pigeon.getTemperature();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.totalDistanceYaw = Rotation2d.fromDegrees(totalDistanceYaw);
        inputs.pitchPosition = Rotation2d.fromDegrees(pitchPosition.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(rollPosition.getValueAsDouble());
        inputs.tempCelcius = tempCelcius.getValueAsDouble();
    }
}