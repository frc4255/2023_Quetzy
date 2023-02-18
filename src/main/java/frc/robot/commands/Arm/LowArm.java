package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class LowArm extends CommandBase {
  private final Arm m_Arm;
  private final Wrist m_Wrist;

  public LowArm(Arm m_Arm, Wrist m_Wrist) {
    this.m_Arm = m_Arm;
    this.m_Wrist = m_Wrist;
    addRequirements(m_Arm, m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.setL1();
    m_Arm.setL1();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}