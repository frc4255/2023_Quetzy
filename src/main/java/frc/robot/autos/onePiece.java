package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.autos.autoCommands.AutoHighArm;
import frc.robot.commands.otherIntakerun;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class onePiece extends SequentialCommandGroup {
    public onePiece(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Intake s_Intake){
        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new AutoHighArm(s_Arm, s_Wrist),
            new otherIntakerun(s_Intake)
        );
    }
}