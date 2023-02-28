package frc.robot.autos.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
public class WristL3 extends CommandBase{
    
    private final Wrist m_Wrist;

    public WristL3(Wrist m_Wrist) {
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
    m_Wrist.setL3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        if (m_Wrist.isNearGoal("high")) {
            return true;
        } else {
            return false;
        }
    }
}
