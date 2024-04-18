package com.team1533.frc.robot.commands.automations;

import java.time.Instant;
import java.util.function.Supplier;

import com.team1533.frc.robot.commands.indexer.GroundToIntake;
import com.team1533.frc.robot.subsystems.leds.Leds;
import com.team1533.frc.robot.subsystems.rollers.Intake;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure;
import com.team1533.frc.robot.subsystems.superstructure.Superstructure.Goal;
import com.team1533.frc.robot.subsystems.swerve.SwerveDrive;
import com.team1533.frc.robot.subsystems.vision.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPickupFieldRelative extends SequentialCommandGroup {

        SwerveDrive m_swerve;
        Superstructure m_Superstructure;
        Intake m_intake;
        Leds m_Leds;
        Vision m_Vision;
        Pose2d note2d;

        public AutoPickupFieldRelative(SwerveDrive m_swerve, Superstructure m_Superstructure, Intake m_intake,
                        Leds m_Leds, Vision m_Vision,
                        Supplier<Pose2d> notePoseFieldRelative) {
                addRequirements(m_intake, m_swerve, m_Superstructure, m_Leds);

                this.m_swerve = m_swerve;
                this.m_Superstructure = m_Superstructure;
                this.m_intake = m_intake;
                this.m_Leds = m_Leds;
                this.m_Vision = m_Vision;
                this.note2d = notePoseFieldRelative.get();

                addCommands(
                                new InstantCommand(() -> m_Superstructure.setGoalCommand(Goal.GROUND_TELEOP),
                                                m_Superstructure),
                                new InstantCommand(() -> m_swerve
                                                .setAutoAlignController(notePoseFieldRelative.get())),
                                new GroundToIntake(m_intake,
                                                m_Leds)
                );
        }
}