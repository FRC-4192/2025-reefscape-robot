// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(DriverConstants.controllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.controllerPort);
    private final CommandXboxController tester = new CommandXboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    // private final int intakeLeftTrigger = XboxController.Axis.kLeftTrigger.value;
    // private final int intakeRightTrigger = XboxController.Axis.kRightTrigger.value;

    /* Operator Controls */
    // private final int shoot = XboxController.Axis.kLeftTrigger.value;
    // private final int index = XboxController.Axis.kRightTrigger.value;
    // private final int shooterWristAdjustment = XboxController.Axis.kRightY.value;
    // private final int elevatorAdjustment = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final Trigger robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final Trigger slowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final Trigger targetFollow = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final Trigger wristToggle = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final Trigger climbWristPosition = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Operator Buttons */
    // private final Trigger elevatorRest = new POVButton(operator, 180); // down
    // private final Trigger elevatorAmp = new POVButton(operator,  90); // right
    // private final Trigger elevatorShoot = new POVButton(operator, 270); // left
    // private final Trigger elevatorTrap = new POVButton(operator, 0); // up
    // private final Trigger shooterOn = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final Trigger shooterWristRest = new POVButton(operator, 180);
    // private final Trigger shooterWristSpeaker = new POVButton(operator, 90);
    // private final Trigger shooterWristPodium = new POVButton(operator, 0);
    // private final Trigger shooterWristAmp = new POVButton(operator, 270);
    // private final Trigger shooterWristRegression = new JoystickButton(operator, XboxController.Button.kY.value);
    // private final Trigger shooterWristRezero = new JoystickButton(operator, XboxController.Button.kB.value);
    //private final Trigger shooterWristUp = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    //private final Trigger shooterWristDown = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final Trigger elevatorAmp = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // private final Trigger elevatorRest = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final Trigger climb = new JoystickButton(operator, XboxController.Button.kX.value);

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveDrive swerve = new SwerveDrive();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);


    // private final SendableChooser<Command> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.setDefaultCommand(new TeleopSwerve(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            driver::getLeftBumperButtonReleased,
            () -> driver.getRightBumperButton() || driver.getLeftStickButton() || driver.getRightStickButton()
        ));
        // swerve.setDefaultCommand(new TeleopSwerve(
        //     swerve,
        //     driver::getLeftY,
        //     driver::getLeftX,
        //     operator::getRightX,
        //     operator.leftBumper(),
        //     operator.rightBumper().or(operator.leftStick()).or(operator.rightStick())
        // ));

        // elevator.setDefaultCommand(new FunctionalCommand(
        //     () -> {},
        //     () -> elevator.runRaw(-operator.getLeftY()),
        //     (x) -> elevator.runRaw(0),
        //     () -> false,
        //     elevator
        // ));

        arm.setDefaultCommand(new FunctionalCommand(
            () -> {},
            () -> { arm.runTakeRaw(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis()); arm.runWristRaw(operator.getRightY()); },
            (x) -> { arm.runTakeRaw(0); arm.runWristRaw(0); },
            () -> false,
            arm
        ));

        // Configure the trigger bindings
        configureBindings();

        // // For convenience a programmer could change this when going to competition.
        // boolean isCompetition = true;

        // // Build an auto chooser. This will use Commands.none() as the default option.
        // // As an example, this will only show autos that start with "comp" while at
        // // competition as defined by the programmer
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        // (stream) -> isCompetition
        //     ? stream.filter(auto -> auto.getName().startsWith("comp"))
        //     : stream
        // );

        // autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        // configureTester();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

        new Trigger(driver::getYButtonPressed).onTrue(new InstantCommand(swerve::zeroHeading, swerve));
        // driver.leftBumper().onTrue(new InstantCommand(swerve::))
        operator.a().and(operator.b()).getAsBoolean();

        operator.povDown().onFalse(new InstantCommand(() -> elevator.setTarget(Elevator.State.L1)));
        operator.povRight().onFalse(new InstantCommand(() -> elevator.setTarget(Elevator.State.L2)));
    }

    private void configureTester() {
        // elevator
        tester.leftBumper().and(tester.a()).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tester.leftBumper().and(tester.b()).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        tester.leftBumper().and(tester.x()).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        tester.leftBumper().and(tester.y()).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // arm
        tester.rightBumper().and(tester.a()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tester.rightBumper().and(tester.b()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        tester.rightBumper().and(tester.x()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        tester.rightBumper().and(tester.y()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
