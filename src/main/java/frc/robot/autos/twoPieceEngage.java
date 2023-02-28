package frc.robot.autos;

import frc.robot.commands.BottomNode;
import frc.robot.commands.HighNode;
import frc.robot.commands.Stow;
import frc.robot.commands.otherIntakerun;
import frc.robot.commands.runIntake;
import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class twoPieceEngage extends SequentialCommandGroup {

    public twoPieceEngage(Swerve s_Swerve, Intake s_Intake, Arm s_Arm, Wrist s_Wrist){
        PathPlannerTrajectory path1 = PathPlanner.loadPath("2PE-1", 2, 1);
        PathPlannerTrajectory path2 = PathPlanner.loadPath("2PE-2", 2, 1);
       // PathPlannerTrajectory path3 = PathPlanner.loadPath("2PE-3", 2, 1);

        PPSwerveControllerCommand grab1Cone = s_Swerve.followTrajectoryCommand(path1);
        PPSwerveControllerCommand alignToCube = s_Swerve.followTrajectoryCommand(path2);
        PathPlannerTrajectory goToChargeStation = PathPlanner.loadPath("2PE-2", 2, 1);

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new InstantCommand(() -> s_Swerve.resetOdometry(path1.getInitialHolonomicPose())),
            new HighNode(s_Arm, s_Wrist),
            new runIntake(s_Intake).repeatedly().withTimeout(0.1),
            new ParallelCommandGroup(
                new Stow(s_Arm, s_Wrist),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new ParallelCommandGroup(
                        grab1Cone,
                        new SequentialCommandGroup(
                            new WaitCommand(2.85),
                            new BottomNode(s_Arm, s_Wrist),
                            new otherIntakerun(s_Intake).repeatedly().withTimeout(0.2)
                        )
                    )
                )
            ),
            new ParallelCommandGroup(
                new Stow(s_Arm, s_Wrist),
                alignToCube
            ),
            new HighNode(s_Arm, s_Wrist),
            new otherIntakerun(s_Intake)
        );
    }
}