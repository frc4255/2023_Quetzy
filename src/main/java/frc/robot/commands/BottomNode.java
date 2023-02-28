package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.LowArm;
import frc.robot.commands.Wrist.LowWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class BottomNode extends ParallelCommandGroup{
    public BottomNode(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new LowArm(s_Arm),
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new LowWrist(s_Wrist)
            )
        );
    }
}
