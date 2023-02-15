package frc.robot.autos.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlannerTrajectory;
public class followTrajectoryCommand extends CommandBase{
    
    private final Swerve m_Swerve;
    private final PathPlannerTrajectory traj;

    public followTrajectoryCommand(Swerve m_Swerve, PathPlannerTrajectory traj) {
        this.m_Swerve = m_Swerve;
        this.traj = traj;

        //addRequirements(m_Swerve);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Swerve.followTrajectoryCommand(traj);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
