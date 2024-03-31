package com.team1533.lib.control;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;

public class HeadingController {

    private SwerveDrive m_swerve;

    public Pose2d centerOfGoal;

    public enum HeadingControllerState {
        OFF, SNAP, // for snapping to specific headings
        MAINTAIN, // maintaining current heading while driving
    }

    private final PIDController m_PIDController;
    private Supplier<Rotation2d> m_Setpoint;

    @AutoLogOutput
    @Getter
    @Setter
    private HeadingControllerState m_HeadingControllerState = HeadingControllerState.OFF;

    public HeadingController(SwerveDrive m_swerve) {
        this.m_swerve = m_swerve;
        m_PIDController = new PIDController(0, 0, 0);
        m_PIDController.setTolerance(SwerveConstants.RotationConfigs.kSwerveHeadingControllerErrorTolerance);
        m_PIDController.setIZone(10);
        m_PIDController.enableContinuousInput(0, 360);

    }

    /**
     * @param goal_pos pos in degrees
     */
    public void setGoal(Rotation2d goal_pos) {
        m_Setpoint = () -> goal_pos;
    }

    /**
     * @param goal_pos pos supplier in degrees
     */
    public void setGoal(Supplier<Rotation2d> supplier) {
        m_Setpoint = supplier;
    }

    public Rotation2d getGoal() {
        return m_Setpoint.get();
    }

    public boolean isAtGoal() {
        return m_PIDController.atSetpoint();
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update() {
        m_PIDController.setSetpoint(m_Setpoint.get().getDegrees());

        double maxOutput = Double.POSITIVE_INFINITY;

        if (isAtGoal() && getM_HeadingControllerState() == HeadingControllerState.SNAP) {
            m_HeadingControllerState = HeadingControllerState.MAINTAIN;
        }
        switch (m_HeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                m_PIDController.setPID(SwerveConstants.RotationConfigs.kSnapSwerveHeadingKp,
                        SwerveConstants.RotationConfigs.kSnapSwerveHeadingKi,
                        SwerveConstants.RotationConfigs.kSnapSwerveHeadingKd);
                break;
            case MAINTAIN:
                m_PIDController.setPID(SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKpHighVelocity,
                        SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKiHighVelocity,
                        SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKdHighVelocity);
                maxOutput = 1.0;
                break;
        }

        return MathUtil.clamp(m_PIDController.calculate(m_swerve.getPose().getRotation().getDegrees()), -maxOutput,
                maxOutput);
    }
}