package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class runIntake extends CommandBase {

  private final Intake m_Intake;
  private final RobotContainer m_robotContainer;
  private final RobotState s_RobotState;
  private final Timer m_Timer;

  private boolean isFinished = false;

  public runIntake(Intake m_Intake, RobotContainer m_Robotcontainer, RobotState s_RobotState) {
    this.m_Intake = m_Intake;
    this.m_robotContainer = m_Robotcontainer;
    this.s_RobotState = s_RobotState;

    m_Timer = new Timer();

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.intakeObject();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Intake.hasObject()) {
      return;
    }

    m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0.5);
    s_RobotState.toggleState(104);

    Timer.delay(0.3);

    m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0);

    Timer.delay(3);

    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
    s_RobotState.toggleState(101);
    m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
