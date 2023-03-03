package frc.robot.autos;

import frc.robot.subsystems.Arm;
import frc.robot.autos.autoCommands.ArmL3;
import frc.robot.autos.autoCommands.WristL3;
import frc.robot.commands.otherIntakerun;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class onePiece extends SequentialCommandGroup {
    public onePiece(Swerve s_Swerve, Arm s_Arm, Wrist s_Wrist, Intake s_Intake){
        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
            new ParallelCommandGroup(
                new ArmL3(s_Arm),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new WristL3(s_Wrist)
                )
            ),
            new otherIntakerun(s_Intake).repeatedly().withTimeout(0.3)
        );
    }
}