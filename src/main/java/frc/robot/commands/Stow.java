package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.StowArm;
import frc.robot.commands.Wrist.StowWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Stow extends ParallelCommandGroup{
    public Stow(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new StowWrist(s_Wrist),
            new SequentialCommandGroup(
                new WaitCommand(0.4),
                new StowArm(s_Arm)
            )
        );
    }
}
