package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
//Intake cube, extake cone
public class runIntake extends CommandBase {

  private final Intake m_Intake;
  private final RobotContainer m_robotContainer;
  private final Timer m_Timer;

  public runIntake(Intake m_Intake, RobotContainer m_Robotcontainer) {
    this.m_Intake = m_Intake;
    this.m_robotContainer = m_Robotcontainer;

    m_Timer = new Timer();

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.intakeObject();
    m_Timer.reset();
  }

  @Override
  public void execute() {
    if (m_Intake.hasObject()) { //If a cube has been picked up
      m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
    m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
