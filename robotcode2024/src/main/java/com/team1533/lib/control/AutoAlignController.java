package com.team1533.lib.control;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class AutoAlignController {

        public double xAutoSpeed = 0;
        public double yAutoSpeed = 0;
        public double rAutoSpeed = 0;
        public PIDController xController = new PIDController(1, 0.01, 0);
        public PIDController yController = new PIDController(1, 0.01, 0);
        public PIDController omegaController = new PIDController(0.1, 0.1, 0.0025);
        private SwerveDrive m_swerve;
        public Supplier<Pose2d> m_Setpoint;

        public enum AutoAlignControllerState {
                OFF, AUTO_ALIGN_FAST, AUTO_ALIGN_SLOW
        }

        @AutoLogOutput
        @Getter
        @Setter
        private AutoAlignControllerState m_AutoAlignControllerState = AutoAlignControllerState.OFF;

        public AutoAlignController(SwerveDrive m_swerve) {
                this.m_swerve = m_swerve;
                xController.setTolerance(SwerveConstants.AutoAlignConstants.kAutoALignControllerErrorTolerance);
                yController.setTolerance(SwerveConstants.AutoAlignConstants.kAutoALignControllerErrorTolerance);
                omegaController.setTolerance(SwerveConstants.RotationConfigs.kSwerveHeadingControllerErrorTolerance);
                omegaController.setIZone(10);
                omegaController.enableContinuousInput(0, 360);
        }

        /**
         * @param goal_pos pos in degrees
         */
        public void setGoal(Pose2d goal_pos) {
                m_Setpoint = () -> goal_pos;
        }

        /**
         * @param goal_pos pos supplier in degrees
         */
        public void setGoal(Supplier<Pose2d> supplier) {
                m_Setpoint = supplier;
        }

        public Pose2d getGoal() {
                return m_Setpoint.get();
        }

        public boolean isAtFastGoal() {
                return Math.hypot(xController.getPositionError(), yController.getPositionError()) < 1;
                // && omegaController.getPositionError() < 2;
        }

        public boolean isAtSlowGoal() {
                return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
        }

        /**
         * Should be called from a looper at a constant dt
         */
        public ChassisSpeeds update() {
                xController.setSetpoint(m_Setpoint.get().getX());
                yController.setSetpoint(m_Setpoint.get().getY());
                omegaController.setSetpoint(m_Setpoint.get().getRotation().getDegrees());

                double maxTranslationOutput = Double.POSITIVE_INFINITY;
                double maxOmegaOutput = Double.POSITIVE_INFINITY;
            
                
                switch (m_AutoAlignControllerState) {
                        case OFF:
                                return ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.fromDegrees(0));
                        case AUTO_ALIGN_FAST:
                          maxOmegaOutput = 2 * Math.PI;
                                maxTranslationOutput = 3;
                                break;
                        case AUTO_ALIGN_SLOW:
                                maxOmegaOutput = Math.PI;
                                maxTranslationOutput = 0.5;
                                break;
                }
                ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                MathUtil.clamp(xController.calculate(m_swerve.getPose().getX()),
                                                -maxTranslationOutput,
                                                maxTranslationOutput),
                                MathUtil.clamp(yController.calculate(m_swerve.getPose().getY()),
                                                -maxTranslationOutput,
                                                maxTranslationOutput),
                                MathUtil.clamp(omegaController.calculate(m_swerve.getPose().getRotation().getDegrees()),
                                                -maxOmegaOutput,
                                                maxOmegaOutput),
                                m_swerve.getPose().getRotation());
    if (isAtFastGoal() && getM_AutoAlignControllerState() == AutoAlignControllerState.AUTO_ALIGN_FAST) {
                        m_AutoAlignControllerState = AutoAlignControllerState.AUTO_ALIGN_SLOW;
                }

                return desiredSpeeds;
                
        }
}