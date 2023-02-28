package frc.robot.autos.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
public class AutoHighNode extends CommandBase{
    
    private final Arm m_Arm;
    private final Wrist m_Wrist;
    private boolean hasWaited = false;

    public AutoHighNode(Arm m_Arm, Wrist m_Wrist) {
        this.m_Arm = m_Arm;
        this.m_Wrist = m_Wrist;

        addRequirements(m_Arm, m_Wrist);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setL3();

    if (!hasWaited) {
      Timer.delay(0.5);
      hasWaited = true;
      m_Wrist.enable();
    }

    m_Wrist.setL3();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrist.disable();
    m_Arm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
