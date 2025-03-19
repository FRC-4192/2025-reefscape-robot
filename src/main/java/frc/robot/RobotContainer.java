// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TargetAlign;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TimedCommand;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Glitter;
import frc.robot.subsystems.RampTake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Take;

import java.time.Duration;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final CommandXboxController driverC = new CommandXboxController(DriverConstants.controllerPort);
    private final XboxController op = new XboxController(OperatorConstants.controllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.controllerPort);
    private final CommandXboxController tester = new CommandXboxController(2);
  
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveDrive swerve = new SwerveDrive();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final Take take = new Take();
    private final RampTake rampTake = new RampTake();
    private final Glitter glitter = new Glitter();

    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerve.setDefaultCommand(new TeleopSwerve(
            swerve,
            glitter,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            driver::getStartButtonPressed,
            () -> driver.getLeftStickButton() || driver.getRightStickButton() || elevator.getTarget() != Elevator.State.L0
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
        elevator.setDefaultCommand(elevator.stay());
        // if(operator.getRightY()>.05||operator.getRightY()<-.05)
        arm.setDefaultCommand(arm.stay());

        // arm.setDefaultCommand(new FunctionalCommand(
        //     () -> {},
        //     () -> { arm.runTakeRaw(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis()); /*arm.runWristRaw(operator.getRightY());*/ },
        //     (x) -> { arm.runTakeRaw(0); /*arm.runWristRaw(0);*/ },
        //     () -> false,
        //     arm
        // ));

        //New code before comp on sunday -- delete this comment after
        // arm.setDefaultCommand(new FunctionalCommand(
        //     () -> {},
        //     () -> { arm.adjust(operator.getRightY()); },
        //     (x) -> { arm.adjust(0); },
        //     () -> false,
        //     arm
        // ));
        take.setDefaultCommand(take.runTake(() -> .75 * Math.min(Math.max(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis() + driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(), -1), 1)));
        rampTake.setDefaultCommand(rampTake.runTake(() -> .5 * (operator.getRightTriggerAxis() - operator.getLeftTriggerAxis())));

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
        boolean useArm = false;
        boolean useElevator = true;
        boolean useIntake = false;

        NamedCommands.registerCommand("arm hold", useArm ? arm.setTarget2(Arm.State.HOLDING) : new InstantCommand());
        NamedCommands.registerCommand("arm intake", useArm ? arm.setTarget2(Arm.State.INTAKING) : new InstantCommand());
        NamedCommands.registerCommand("elevator L4", useElevator ? elevator.setTarget2(Elevator.State.L4) : new InstantCommand());
        NamedCommands.registerCommand("score", new SequentialCommandGroup(
            useArm ? arm.setTarget(Arm.State.SCORING).raceWith(new WaitCommand(1)) : new InstantCommand(),
            useIntake ? take.runOuttake(()-> 1).raceWith(new WaitCommand(1.5)) : new InstantCommand(),
            useIntake ? take.runOuttake(()-> 0).raceWith(new WaitCommand(.3)) : new InstantCommand(),
            useArm ? arm.setTarget(Arm.State.HOLDING).raceWith(new WaitCommand(1)) : new InstantCommand(),
            useElevator ? elevator.setTarget2(Elevator.State.L0) : new InstantCommand()
        ));
        NamedCommands.registerCommand("intakeCoral", new SequentialCommandGroup(
            useIntake ? take.runIntake(()-> 1).raceWith(new WaitCommand(1.5)).alongWith(rampTake.runIntake(()-> 1).raceWith(new WaitCommand(1.5))) : new InstantCommand(),
            useIntake ? take.runIntake(()-> 0).raceWith(new WaitCommand(.3)).alongWith(rampTake.runIntake(()-> 0).raceWith(new WaitCommand(.3))) : new InstantCommand()
        
        ));
        NamedCommands.registerCommand("alignToReefL", true ? new TargetAlign(swerve, false).raceWith(new WaitCommand(3)) : new InstantCommand());
        NamedCommands.registerCommand("alignToReefR", true ? new TargetAlign(swerve, true).raceWith(new WaitCommand(3)) : new InstantCommand());
        // NamedCommands.registerCommand("score", new InstantCommand());
        // NamedCommands.registerCommand("score", new SequentialCommandGroup(
        //         take.runOuttake(()->1).raceWith(new WaitCommand(1.5)),
        //         take.runOuttake(()->0).raceWith(new WaitCommand(.3))
        //  ));

        // NamedCommands.registerCommand("Knock off algae", new SequentialCommandGroup(
        //     arm.setTarget( Arm.State.INTAKING ),
        //     elevator.setTarget( Elevator.State.L4 )
        // ));
        // NamedCommands.registerCommand("deposit coral on L4", new SequentialCommandGroup(
        //     arm.setTarget( Arm.State.SCORING ),
        //     new TimedCommand( take.runOuttake( ()->1) , 3000 )
        // ));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("elevator test", new SequentialCommandGroup(
            elevator.setTarget2(Elevator.State.L4).asProxy(),
            new WaitCommand(1.5),
            elevator.setTarget2(Elevator.State.L0).asProxy(),
            new WaitCommand(1.5),
            elevator.setTarget2(Elevator.State.L4).asProxy(),
            new WaitCommand(1.5),
            elevator.setTarget2(Elevator.State.L0).asProxy()
        ));
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        

        // operator.povLeft().whileTrue(new TargetAlign(swerve));
        // new Trigger(() -> driver.getPOV() == 270).whileTrue(new TargetAlign(swerve, false));
        // new Trigger(() -> driver.getPOV() == 90).whileTrue(new TargetAlign(swerve, true));
        driverC.leftBumper().whileTrue(new TargetAlign(swerve, false));
        driverC.rightBumper().whileTrue(new TargetAlign(swerve, true));
        
        

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

        driverC.back().onTrue(swerve.runOnce(swerve::zeroHeading));
        // driver.leftBumper().onTrue(new InstantCommand(swerve::))
        // operator.a().and(operator.b()).getAsBoolean();


        operator.povDown().or(driverC.povDown()).and(arm::isSafeToLift).onTrue(elevator.setTarget2(Elevator.State.L0));
        operator.povRight().or(driverC.povLeft()).and(arm::isSafeToLift).onTrue(elevator.setTarget2(Elevator.State.L3));
        operator.povUp().or(driverC.povUp()).and(arm::isSafeToLift).onTrue(elevator.setTarget2(Elevator.State.L4));

        // operator.povLeft().or(() -> driver.getPOV() == 270).whileTrue(glitter.intakeReady());
        // operator.povLeft().or(() -> driver.getPOV() == 270).whileFalse(glitter.intakeNotReady());

        // operator.a().onTrue(elevator.setTarget(Elevator.State.L1));
        // operator.a().onTrue(arm.setTarget(Arm.State.INTAKING));
        // operator.a().onTrue(arm.setRawTake(1));

        // operator.b().onTrue(elevator.setTarget(Elevator.State.L3));
        // operator.b().onTrue(arm.setTarget(Arm.State.HOLDING));
        // operator.b().onTrue(arm.setRawTake(0));


  
        operator.a().or(() -> driver.getAButton()).and(elevator::safeToIntake).onTrue(arm.setTarget(Arm.State.INTAKING));
        operator.b().or(() -> driver.getBButton()).onTrue(arm.setTarget(Arm.State.SCORING));
        operator.x().or(() -> driver.getXButton()).onTrue(arm.setTarget(Arm.State.HOLDING));

        operator.leftStick().whileTrue(arm.rezero());
        operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, .05).whileTrue(arm.adjust(-operator.getRightY()).finallyDo(arm::stay));

        // operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, .1).whileTrue(arm.adjust(-.1*operator.getRightY()));
        
        // operator.leftBumper().onTrue(glitter.dereasePWM()
        //     .alongWith(new TimedCommand(() -> operator.setRumble(RumbleType.kLeftRumble, 1), 500))
        //     .andThen(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
        // operator.rightBumper().onTrue(glitter.increasePWM()
        //     .alongWith(new TimedCommand(() -> operator.setRumble(RumbleType.kLeftRumble, 1), 500))
        //     .andThen(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
    }

    private void configureTester() {
        // elevator
        // tester.leftBumper().and(tester.a()).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // tester.leftBumper().and(tester.b()).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // tester.leftBumper().and(tester.x()).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // tester.leftBumper().and(tester.y()).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        elevator.setDefaultCommand(elevator.runRawg(() -> -tester.getLeftY()));

        // arm
        // tester.rightBumper().and(tester.a()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // tester.rightBumper().and(tester.b()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // tester.rightBumper().and(tester.x()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // tester.rightBumper().and(tester.y()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        arm.setDefaultCommand(arm.run(() -> arm.runWristRaw(-tester.getRightY())));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
        // PathPlannerPath path = new PathPlannerPath(
        //     PathPlannerPath.waypointsFromPoses(
        //         new Pose2d(30, 0, Rotation2d.kZero),
        //         new Pose2d(30, 30, Rotation2d.k180deg)
        //     ),
        //     new PathConstraints(5,5, 10, 10),
        //     null,
        //     new GoalEndState(1, Rotation2d.k180deg)
        // );
        // PathPlannerTrajectory traj = path.generateTrajectory(new ChassisSpeeds(), Rotation2d.kZero, Constants.SwerveConstants.autoPathFollowerConfig);
        // return AutoBuilder.followPath(path).alongWith(new FunctionalCommand(
        //     () -> {},
        //     () -> { SmartDashboard.putString("aut rnd", traj.getState(100).pose.toString()); },
        //     x -> {},
        //     () -> false
        // ));

        // Command armToHold = new FunctionalCommand( () -> {} , ()->arm.setTarget(Arm.State.HOLDING), (x) -> {}, () -> arm.getError()<=2, arm);
        // Command armToHold2 = new FunctionalCommand( () -> {} , ()->arm.setTarget(Arm.State.HOLDING), (x) -> {}, () -> arm.getError()<=2, arm);
        // Command armToScore = new FunctionalCommand( () -> {} , ()->arm.setTarget(Arm.State.SCORING), (x) -> {}, () -> arm.getError()<=2, arm);
        // Command elevatorToL4 = new FunctionalCommand( () -> {} , ()->elevator.setTarget(Elevator.State.L4), (x) -> {}, () -> elevator.getError()<=.02, elevator);
        // Command elevatorToL0 = new FunctionalCommand( () -> {} , ()->elevator.setTarget(Elevator.State.L0), (x) -> {}, () -> elevator.getError()<=.02, elevator);



        // Command armToHold = arm.setTarget(Arm.State.HOLDING).raceWith(new WaitCommand(1));
        // Command armToHold2 = arm.setTarget(Arm.State.HOLDING).raceWith(new WaitCommand(1));
        // Command armToScore = arm.setTarget(Arm.State.SCORING).raceWith(new WaitCommand(1));
        // Command elevatorToL4 = elevator.setTarget(Elevator.State.L4).raceWith(new WaitCommand(2.5));
        // Command elevatorToL0 = elevator.setTarget(Elevator.State.L0).raceWith(new WaitCommand(3));

        // // Command depositCoral = new TimedCommand(take.runOuttake(()->1), 3000).finallyDo(() -> take.runOuttake(() -> 0));
        // Command depositCoral = take.runOuttake(()->1);
        // Command stopOuttake = take.runOuttake(()->0);
        // Command drive = new FunctionalCommand(
        //     () -> swerve.setPose(new Pose2d(7.25, 4.19, Rotation2d.kZero)),
        //     () -> swerve.drive(new ChassisSpeeds(-1,0,0)),
        //     (x) -> swerve.drive(new ChassisSpeeds(0, 0, 0)),
        //     () -> swerve.getPose().relativeTo(new Pose2d(5.25, 4.19, Rotation2d.kZero)).getTranslation().getNorm() <= .05,
        //     swerve
        // ).raceWith(new WaitCommand(4));
        // // Command driveStill = swerve.run(() -> swerve.drive(Pose2d.kZero, false, false));

    

        // return new SequentialCommandGroup(
        //     armToHold.alongWith(drive),
        //     elevatorToL4,
        //     armToScore,
        //     depositCoral.raceWith(new WaitCommand(1.5)),
        //     stopOuttake.raceWith(new WaitCommand(0.3)),
        //     armToHold2,
        //     elevatorToL0
        // );

        // return new TimedCommand(
        //     () -> swerve.drive(new ChassisSpeeds(-1, 0, 0)),
        //     2000,
        //     swerve
        // ).finallyDo(() -> swerve.drive(Pose2d.kZero, false, false));
    }
}
