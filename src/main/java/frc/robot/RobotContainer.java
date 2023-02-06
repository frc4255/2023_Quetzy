package frc.robot;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.HighArm;
import frc.robot.commands.Arm.LowArm;
import frc.robot.commands.Arm.MidArm;
import frc.robot.commands.Arm.ShelfArm;
import frc.robot.commands.Arm.StowArm;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick mechOperator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton runIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    //Mechanism Operator Buttons
    private final JoystickButton setL1 = new JoystickButton(mechOperator, XboxController.Button.kA.value);
    private final JoystickButton setL2 = new JoystickButton(mechOperator, XboxController.Button.kY.value);
    private final JoystickButton setL3 = new JoystickButton(mechOperator, XboxController.Button.kX.value);
    private final JoystickButton setShelf = new JoystickButton(mechOperator, XboxController.Button.kB.value);
    private final JoystickButton setStow = new JoystickButton(mechOperator, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm s_Arm = new Arm();
    private final Wrist s_Wrist = new Wrist();
    private final Intake s_Intake = new Intake();

    private SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        configAutoChooser();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        runIntake.whileTrue(new runIntake(s_Intake));

        //Mech operator bindings
        setL1.onTrue(new LowArm(s_Arm, s_Wrist));
        setL2.onTrue(new MidArm(s_Arm, s_Wrist));
        setL3.onTrue(new HighArm(s_Arm, s_Wrist));
        setShelf.onTrue(new ShelfArm(s_Arm, s_Wrist));
        setStow.onTrue(new StowArm(s_Arm, s_Wrist));
        
    }

    private void configAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", null); //TODO: Create Do Nothing auto object
        autoChooser.setDefaultOption("Test path", new testPath(s_Swerve, s_Intake));
        autoChooser.addOption("L3 Link", null); //TODO: Create L3 Link Class/Object
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}