package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.HighArm;
import frc.robot.commands.Wrist.HighWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class HighNode extends ParallelCommandGroup{
    public HighNode(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new HighArm(s_Arm),
            new SequentialCommandGroup(
                new WaitCommand(0.4),
                new HighWrist(s_Wrist)
            )
        );
    }
}

