package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class runIntake extends CommandBase{
    
    private final Intake m_Intake;

    public runIntake(Intake m_Intake) {
        this.m_Intake = m_Intake;

        addRequirements(m_Intake);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.intakeObject();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
