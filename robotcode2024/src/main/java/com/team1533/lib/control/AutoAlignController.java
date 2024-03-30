package com.team1533.lib.control;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team1533.frc.robot.subsystems.swerve.SwerveConstants;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import com.team1533.frc.robot.util.AllianceFlipUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;

public class AutoAlignController {
        @AutoLogOutput
        @Getter
        public ChassisSpeeds desiredSpeed = new ChassisSpeeds(0, 0, 0);
        public double xAutoSpeed = 0;
        public double yAutoSpeed = 0;
        public double rAutoSpeed = 0;
        public PIDController xController = new PIDController(5, 0.01, 0);
        public PIDController yController = new PIDController(3, 0.01, 0);
        public PIDController omegaController = new PIDController(7, 0.01, 0);
        private SwerveDrive m_SwerveDrive;
        public Supplier<Pose2d> poseSupplier;
        public Pose2d targetPose;

        public enum AutoAlignControllerState {
                OFF, AUTO_ALIGN_FAST, AUTO_ALIGN_SLOW
        }
}