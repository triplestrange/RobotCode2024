package frc.robot.commands.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.indexer.GroundToIntake;
import frc.robot.subsystems.rollers.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoPickupFieldRelative extends SequentialCommandGroup {

    SwerveDrive m_swerve;
    Superstructure m_Superstructure;
    Intake m_intake;
    Translation2d note2d;

    public AutoPickupFieldRelative(SwerveDrive m_swerve, Superstructure m_Superstructure, Intake m_intake,
            Translation2d notePoseFieldRelative) {
        addRequirements(m_intake, m_swerve, m_Superstructure);

        this.m_swerve = m_swerve;
        this.m_Superstructure = m_Superstructure;
        this.m_intake = m_intake;
        this.note2d = notePoseFieldRelative;

        addCommands(
                new InstantCommand(() -> m_Superstructure.setGoalCommand(Goal.GROUND), m_Superstructure),
                new GroundToIntake(m_intake),
                new DriveTo(new Pose2d(note2d, m_swerve.getPose().getTranslation().minus(note2d).getAngle()), 0, 0,
                        m_swerve),
                new InstantCommand(() -> m_Superstructure.setGoalCommand(Goal.STOW))

        );
    }
}