package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.MidArm;
import frc.robot.commands.Wrist.MidWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class MiddleNode extends ParallelCommandGroup{
    public MiddleNode(Arm s_Arm, Wrist s_Wrist) {
        addCommands(
            new MidArm(s_Arm),
            new MidWrist(s_Wrist)
        );
    }
}