package frc.robot.autos;

import frc.robot.RobotContainer;
import frc.robot.autos.autoCommands.autoDock;
import frc.robot.commands.HighNode;
import frc.robot.commands.Stow;
import frc.robot.commands.runIntake;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class onePieceEngage extends SequentialCommandGroup {

    public onePieceEngage(Swerve s_Swerve, Intake s_Intake, Arm s_Arm, Wrist s_Wrist, RobotContainer m_RobotContainer, RobotState s_RobotState){
        PathPlannerTrajectory path = PathPlanner.loadPath("1PE", 4, 3);

        PPSwerveControllerCommand outAndIn = s_Swerve.followTrajectoryCommand(path);

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve),
            new InstantCommand(() -> s_Swerve.resetOdometry(path.getInitialHolonomicPose()), s_Swerve),
            new WaitCommand(0.1),
            new HighNode(s_Arm, s_Wrist),
            new runIntake(s_Intake, m_RobotContainer, s_RobotState).repeatedly().withTimeout(0.1),
            new Stow(s_Arm, s_Wrist),
            outAndIn,
            new autoDock(s_Swerve, s_RobotState)
        );
    }
}