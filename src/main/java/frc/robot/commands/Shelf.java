package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ShelfArm;
import frc.robot.commands.Wrist.ShelfWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Shelf extends ParallelCommandGroup{
    public Shelf(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new ShelfArm(s_Arm),
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new ShelfWrist(s_Wrist)
            )
        );
    }
}

