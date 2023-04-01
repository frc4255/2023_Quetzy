package frc.robot.autos;

import frc.robot.RobotContainer;
import frc.robot.autos.autoCommands.autoDock;
import frc.robot.autos.autoCommands.avoidChargeStation;
import frc.robot.commands.BottomNode;
import frc.robot.commands.HighNode;
import frc.robot.commands.Stow;
import frc.robot.commands.otherIntakerun;
import frc.robot.commands.runIntake;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class threePiece extends SequentialCommandGroup {

    public threePiece(Swerve s_Swerve, Intake s_Intake, Arm s_Arm, Wrist s_Wrist, RobotContainer m_RobotContainer, RobotState s_RobotState){

        PathPlannerTrajectory path1 = PathPlanner.loadPath("2PE-1", new PathConstraints(4, 3));
        PathPlannerTrajectory path2 = PathPlanner.loadPath("3P", new PathConstraints(4, 4));

        PPSwerveControllerCommand swoopAndScoop = s_Swerve.followTrajectoryCommand(path1);
        PPSwerveControllerCommand swoopAndScoop2 = s_Swerve.followTrajectoryCommand(path2);

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve),
            new InstantCommand(() -> s_Swerve.resetOdometry(path1), s_Swerve),
            new WaitCommand(0.1),
            new HighNode(s_Arm, s_Wrist),
            new runIntake(s_Intake, m_RobotContainer).repeatedly().withTimeout(0.1),
            new ParallelCommandGroup(
                new InstantCommand(() -> s_RobotState.setState(RobotState.State.CUBE)),
                swoopAndScoop,
                new SequentialCommandGroup(
                    new Stow(s_Arm, s_Wrist),
                    new BottomNode(s_Arm, s_Wrist),
                    new runIntake(s_Intake, m_RobotContainer).repeatedly().withTimeout(0.95),
                    new Stow(s_Arm, s_Wrist),
                    new WaitCommand(0.5),
                    new HighNode(s_Arm, s_Wrist)
                )
            ),
            new otherIntakerun(s_Intake, m_RobotContainer).repeatedly().withTimeout(0.2),
            new Stow(s_Arm, s_Wrist),
            new ParallelCommandGroup(
                swoopAndScoop2,
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new BottomNode(s_Arm, s_Wrist),
                    new runIntake(s_Intake, m_RobotContainer).repeatedly().withTimeout(0.95),
                    new Stow(s_Arm, s_Wrist),
                    new WaitCommand(0.5),
                    new HighNode(s_Arm, s_Wrist)
                )
            ),
            new otherIntakerun(s_Intake, m_RobotContainer).repeatedly().withTimeout(0.2),
            new Stow(s_Arm, s_Wrist)
        );
    }
}