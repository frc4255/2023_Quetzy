package frc.robot.autos;

import frc.robot.subsystems.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class testPath extends SequentialCommandGroup {

    private Intake s_intake;
    private Swerve s_swerve;

    public testPath(Swerve s_Swerve, Intake s_Intake){

        this.s_intake = s_intake;
        this.s_swerve = s_swerve;
        PathPlannerTrajectory path1 = PathPlanner.loadPath("TestPath1", 2, 1);
        PathPlannerTrajectory path2 = PathPlanner.loadPath("TestPath2", 2, 1);

        PPSwerveControllerCommand follow1 = s_Swerve.followTrajectoryCommand(path1);
        PPSwerveControllerCommand follow2 = s_Swerve.followTrajectoryCommand(path2);

        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new InstantCommand(() -> s_Swerve.resetOdometry(path1.getInitialHolonomicPose())),
            follow1,
            follow2
        );
    }
}