package frc.robot.commands.Arm;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class MidArm extends CommandBase {
  private final Arm m_Arm;
  private final Wrist m_Wrist;

  public MidArm(Arm m_Arm, Wrist m_Wrist) {
    this.m_Wrist = m_Wrist;
    this.m_Arm = m_Arm;
    addRequirements(m_Arm, m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Wrist.setL2();
    //m_Arm.setL2();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setL2();
    m_Wrist.setL2();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command decided to end");
    System.out.println("Error: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}