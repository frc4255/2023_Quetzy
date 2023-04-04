package frc.robot;

import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton runIntake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton otherIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton coneState = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton cubeState = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton setL2 = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton setSingle = new JoystickButton(driver, XboxController.Button.kA.value);
    private final POVButton setStow = new POVButton(driver, 180);
    private final POVButton setL3 = new POVButton(driver, 270);
    private final POVButton setL1 = new POVButton(driver, 0);
    private final POVButton setShelf = new POVButton(driver, 90);

    /* Subsystems */
    private final RobotState s_RobotState = new RobotState();
    private final Swerve s_Swerve = new Swerve(this);
    private final Arm s_Arm = new Arm(s_RobotState);
    private final Wrist s_Wrist = new Wrist(s_RobotState);
    private final Intake s_Intake = new Intake(s_RobotState);

    public SendableChooser<Command> autoChooser;

    private DriverStation.Alliance allianceColor;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();

        configAutoChooser();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        runIntake.whileTrue(new runIntake(s_Intake, this));
        otherIntake.whileTrue(new otherIntakerun(s_Intake, this));
        setStow.onTrue(new Stow(s_Arm, s_Wrist));
        setL2.onTrue(new MiddleNode(s_Arm, s_Wrist));
        setL3.onTrue(new HighNode(s_Arm, s_Wrist));
        setL1.onTrue(new BottomNode(s_Arm, s_Wrist));
        setShelf.onTrue(new Shelf(s_Arm, s_Wrist));
        coneState.onTrue(new InstantCommand(() -> s_RobotState.setState(RobotState.State.CONE)));
        cubeState.onTrue(new InstantCommand(() -> s_RobotState.setState(RobotState.State.CUBE)));
        setSingle.onTrue(new Single(s_Arm, s_Wrist));
    }

    private void configAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("2PE", new twoPieceEngage(s_Swerve, s_Intake, s_Arm, s_Wrist, this, s_RobotState));
        autoChooser.addOption("1PE", new onePieceEngage(s_Swerve, s_Intake, s_Arm, s_Wrist, this, s_RobotState));
        autoChooser.addOption("2P", new twoPiece(s_Swerve, s_Intake, s_Arm, s_Wrist, this, s_RobotState));
        autoChooser.addOption("3P", new threePiece(s_Swerve, s_Intake, s_Arm, s_Wrist, this, s_RobotState));
        autoChooser.addOption("Do Nothing", null);

        SmartDashboard.putData(autoChooser);
    }

    public void disableStuffFromAuto() {
        s_Arm.disable();
        s_Wrist.disable();
    }

    public void ConeState() {
        s_RobotState.setState(RobotState.State.CONE);
    }

    public Joystick getController() {
        return driver;
    }

    public void setAllianceColor() {
        allianceColor = DriverStation.getAlliance();
    }

    public DriverStation.Alliance getAllianceColor() {
        return allianceColor;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new testPath(s_Swerve, s_Intake);

        return autoChooser.getSelected();
    }
}