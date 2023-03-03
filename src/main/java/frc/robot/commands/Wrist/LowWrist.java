package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
public class LowWrist extends CommandBase{
    
    private final Wrist m_Wrist;

    public LowWrist(Wrist m_Wrist) {
        this.m_Wrist = m_Wrist;

        addRequirements(m_Wrist);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrist.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.setL1();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
    public boolean isFinished() {
      if (m_Wrist.isNearGoal("low")) {
        return true;
      } else {
        return false;
      }
    }
}
