package com.team1533.lib.control;

import com.team1533.frc.robot.commands.automations.Shoot;
import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class HeadingController {

    private Shoot shoot;
    private SwerveDrive m_swerve;

    public Pose2d centerOfGoal;

    public enum HeadingControllerState {
        OFF, SNAP, // for snapping to specific headings
        MAINTAIN, // maintaining current heading while driving
        SPEAKER_MAINTAIN, // for maintaining heading toward origin
        SPEAKER_SNAP, // for snapping heading toward origin
    }

    private final PIDController m_PIDController;
    private double m_Setpoint = 0.0;

    private HeadingControllerState m_HeadingControllerState = HeadingControllerState.OFF;

    public HeadingController(Shoot shoot, SwerveDrive m_swerve) {
        this.shoot = shoot;
        this.m_swerve = m_swerve;
        m_PIDController = new PIDController(0, 0, 0);
        m_PIDController.setTolerance(SwerveConstants.RotationConfigs.kSwerveHeadingControllerErrorTolerance);
    }

    public HeadingControllerState getHeadingControllerState() {
        return m_HeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        m_HeadingControllerState = state;
    }

    /**
     * @param goal_pos pos in degrees
     */
    public void setGoal(double goal_pos) {
        m_Setpoint = goal_pos;
    }

    public double getGoal() {
        return m_Setpoint;
    }

    public boolean isAtGoal() {
        return m_PIDController.atSetpoint();
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double current_angle) {
        m_PIDController.setSetpoint(m_Setpoint);
        m_PIDController.enableContinuousInput(0, 360);

        double maxOutput = Double.POSITIVE_INFINITY;

        var current_translational_velocity = Math.hypot(m_swerve.currentMovement.vxMetersPerSecond,
                m_swerve.currentMovement.vyMetersPerSecond);
        final double kMinTranslationalVelocity = 0.2;
        if (current_translational_velocity < kMinTranslationalVelocity) {
            current_translational_velocity = kMinTranslationalVelocity;
        }
        final double kMaxTranlationalVelocity = 2.5;
        if (current_translational_velocity > kMaxTranlationalVelocity) {
            current_translational_velocity = kMaxTranlationalVelocity;
        }
        double interp = (current_translational_velocity - kMinTranslationalVelocity) / kMaxTranlationalVelocity;

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
            case SPEAKER_MAINTAIN:
                m_PIDController.setPID(
                        SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKpHighVelocity,
                        SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKiHighVelocity,
                        SwerveConstants.RotationConfigs.kMaintainSwerveHeadingKdHighVelocity);
                break;
            case SPEAKER_SNAP:
                m_PIDController.setPID(SwerveConstants.RotationConfigs.kSnapSwerveHeadingKp,
                        SwerveConstants.RotationConfigs.kSnapSwerveHeadingKi,
                        SwerveConstants.RotationConfigs.kSnapSwerveHeadingKd);
                break;
        }

        return MathUtil.clamp(m_PIDController.calculate(current_angle), -maxOutput, maxOutput);
    }
}