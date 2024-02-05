package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.JoystickButtons;
import frc.robot.subsystems.cannon.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;

public class NoteToIntake extends Command {
    private Intake m_Intake;
    private Conveyor m_Conveyor;

    /**
     * Creates a new Drive.
     * 
     * normal = 2.5
     * slow = 0.75
     */
    public NoteToIntake(Intake m_Intake, Conveyor m_Conveyor) {
        addRequirements(m_Intake, m_Conveyor);
        this.m_Intake = m_Intake;
        this.m_Conveyor = m_Conveyor;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Intake.runIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Intake.getIntakeSensor();
    }
}
