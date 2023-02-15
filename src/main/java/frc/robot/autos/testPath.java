package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.autos.autoCommands.followTrajectoryCommand;
import frc.robot.subsystems.*;

import java.util.List;
import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class testPath extends SequentialCommandGroup {

    private Intake s_intake;
    private Swerve s_swerve;

    public testPath(Swerve s_Swerve, Intake s_Intake){

        this.s_intake = s_intake;
        this.s_swerve = s_swerve;
        PathPlannerTrajectory path1 = PathPlanner.loadPath("Test1", 1, 1);
        PathPlannerTrajectory path2 = PathPlanner.loadPath("Test2", 1, 1);

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