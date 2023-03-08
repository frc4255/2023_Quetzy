package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.RobotState.robotStates;
import frc.robot.RobotContainer;

//Intake cone, extake cube
public class otherIntakerun extends CommandBase{
    
    private final Intake m_Intake;
    private final RobotContainer m_robotContainer;
    private final RobotState s_RobotState;
    private final Timer m_Timer;

    public otherIntakerun(Intake m_Intake, RobotContainer m_robotContainer, RobotState s_RobotState) {
        this.m_Intake = m_Intake;
        this.m_robotContainer = m_robotContainer;
        this.s_RobotState = s_RobotState;

        m_Timer = new Timer();

        addRequirements(m_Intake);
    }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.extakeObject();
    m_Timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Intake.hasObject()) {
      m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0.5);
      s_RobotState.toggleState(104);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
    m_robotContainer.getController().setRumble(RumbleType.kBothRumble, 0);

    if (s_RobotState.getLastState() == robotStates.CUBE) {
      s_RobotState.toggleState(101);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
