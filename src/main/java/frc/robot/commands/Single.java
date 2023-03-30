package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.SingleArm;
import frc.robot.commands.Wrist.SingleWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Single extends ParallelCommandGroup{
    public Single(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new SingleArm(s_Arm),
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new SingleWrist(s_Wrist)
            )
        );
    }
}

