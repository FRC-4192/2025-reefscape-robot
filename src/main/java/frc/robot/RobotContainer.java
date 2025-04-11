// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LimelightHelpers;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScore;
import frc.robot.commands.TargetAlign;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

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
    private final SwerveDrive swerve = new SwerveDrive();
    private final GroundTake groundTake = new GroundTake();
    private final GroundWrist groundWrist = new GroundWrist();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final Take take = new Take();
    // private final RampTake rampTake = new RampTake();
    private final Glitter glitter = new Glitter();

    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        TeleopSwerve teleopSwerve = new TeleopSwerve(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            glitter::isFieldCentric,
            () -> glitter.isSpeed() || elevator.getPosition().in(Units.Meters) > .5
        );
        swerve.setDefaultCommand(teleopSwerve);

        elevator.setDefaultCommand(elevator.stayPID());
        arm.setDefaultCommand(arm.stayPID());

        take.setDefaultCommand(take.runIntake(
                () -> .75 * Math.min(Math.max(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis() + driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(), -1), 1)
        ));
        // ground.setDefaultCommand(ground.runIntake(
        //         () -> .75 * Math.min(Math.max(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis() + driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(), -1), 1)
        // ));

        groundTake.setDefaultCommand(groundTake.runIntake(() -> -.95 * operator.getRightY()));

        groundWrist.setDefaultCommand(groundWrist.stayPID());

        // rampTake.setDefaultCommand(rampTake.runTake(() -> .60 * Math.min(Math.max(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis() + driver.getRightTriggerAxis() - driver.getLeftTriggerAxis(), -1), 1)));

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
        boolean useArm = true;
        boolean useElevator = true;
        boolean useIntake = true;

        NamedCommands.registerCommand("arm hold", useArm ? arm.setTargetOnly(Arm.State.HOLDING).asProxy() : new InstantCommand());
        NamedCommands.registerCommand("arm intake", useArm ? arm.setTargetOnly(Arm.State.INTAKING).asProxy() : new InstantCommand());
        NamedCommands.registerCommand("elevator L4", useElevator ? elevator.setTargetOnly(Elevator.State.L4).asProxy() : new InstantCommand());
        NamedCommands.registerCommand("elevator L1", useElevator ? elevator.setTargetOnly(Elevator.State.L1).asProxy() : new InstantCommand());
        NamedCommands.registerCommand("elevator L0", useElevator ? elevator.setTargetOnly(Elevator.State.L0).asProxy() : new InstantCommand());
        NamedCommands.registerCommand("score", new SequentialCommandGroup(
            // take.runTakeOnce(0).asProxy().alongWith(rampTake.runTakeOnce(0).asProxy()),
            useArm ? arm.setTargetOnly(Arm.State.SCORING).asProxy() : new InstantCommand(),
            useIntake ? take.runOuttake(()-> 1).raceWith(new WaitCommand(0.6)).asProxy() : new InstantCommand(),
            useIntake ? take.runTakeOnce(0).raceWith(Commands.waitSeconds(.01)).asProxy() : new InstantCommand(),
            useArm ? arm.setTargetOnly(Arm.State.HOLDING).asProxy().alongWith(elevator.setTargetOnly(Elevator.State.L0).asProxy()) : new InstantCommand()
            // useElevator ? elevator.setTarget2(Elevator.State.L0).asProxy() : new InstantCommand()
        ));
        NamedCommands.registerCommand("score mini", new SequentialCommandGroup(
            useArm ? arm.setTargetOnly(Arm.State.SCORING).asProxy() : new InstantCommand(),
            useIntake ? take.runOuttake(()-> 1).raceWith(new WaitCommand(0.5)).asProxy() : new InstantCommand(),
            useIntake ? take.runTakeOnce(0).raceWith(Commands.waitSeconds(.01)).asProxy() : new InstantCommand()
        ));
        NamedCommands.registerCommand("intakeCoral", useIntake ? take.intakeCoral(() -> .6)/* .raceWith(rampTake.runIntake(() -> .3))*/.asProxy() : new InstantCommand() );
        NamedCommands.registerCommand("intake idle", /*useIntake ? rampTake.runIntake(() -> -.075).asProxy() :*/ new InstantCommand());
        NamedCommands.registerCommand("alignToReefL", true ? new TargetAlign(swerve, false).raceWith(new WaitCommand(1.5)) : new InstantCommand());
        NamedCommands.registerCommand("alignToReefR", true ? new TargetAlign(swerve, true).raceWith(new WaitCommand(1.5)) : new InstantCommand());
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
            elevator.setTargetOnly(Elevator.State.L4).asProxy(),
            new WaitCommand(1.5),
            elevator.setTargetOnly(Elevator.State.L0).asProxy(),
            new WaitCommand(1.5),
            elevator.setTargetOnly(Elevator.State.L4).asProxy(),
            new WaitCommand(1.5),
            elevator.setTargetOnly(Elevator.State.L0).asProxy()
        ));
        // autoChooser.addOption("take test", Commands.sequence(
        //     take.runIntake(() -> .7).alongWith(rampTake.runIntake(() -> .4)).raceWith(Commands.waitSeconds(3)),
        //     Commands.waitSeconds(1),
        //     take.runIntake(() -> 0).alongWith(rampTake.runIntake(() -> 0))
        // ));
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
        driverC.start().onTrue(Commands.runOnce(glitter::toggleDrivePov));
        driverC.leftStick().onTrue(Commands.runOnce(glitter::toggleDriveSpeed));

        driverC.rightStick().whileTrue(swerve.run(swerve::lockDrive));


        // comment out this code when testing
        // new Trigger(() -> driver.getLeftTriggerAxis()>.01).and(() -> elevator.getState() == Elevator.State.L0)
        //     .and(() -> arm.getState()!=Arm.State.INTAKING)
        //     .and(()-> driver.getRightStickButton())
        //         .onTrue(
        //         ground.setTargetOnly(GroundTake.State.SCORING).andThen(ground.outtakeCoral(1))
        //         );

        
        // driverC.y().onTrue((ground.getState()==GroundTake.State.HOLDING) ? ground.setTargetOnly(GroundTake.State.INTAKING) : ground.setTargetOnly(GroundTake.State.HOLDING));
        
        // new Trigger(() -> groundWrist.getState() == GroundWrist.State.INTAKING).whileTrue(groundTake.intakeCoral(
        //     () -> .75 * -operator.getRightY()
        // ).andThen(groundWrist.setTargetOnly(GroundWrist.State.SCORING).asProxy()));
        // new Trigger(() -> groundWrist.getState() == GroundWrist.State.SCORING).whileTrue(groundTake.outtakeCoral(() -> 1 * operator.getRightY()));

        operator.rightBumper().onTrue(groundWrist.setTargetOnly(GroundWrist.State.SCORING));
        operator.leftBumper().onTrue(groundWrist.setTargetOnly(GroundWrist.State.INTAKING));
        driverC.povRight().onTrue(groundWrist.toggleState());
        

        // new Trigger(() -> ground.getState() == GroundTake.State.INTAKING).and(() -> driver.getRightTriggerAxis()>.01).and(()-> driver.getRightStickButton())

        //     .onTrue(ground.intakeCoral(() -> 1));

        // new Trigger(() -> driver.getRightTriggerAxis()>.01)
        //     .and(() -> !ground.hasCoral() )
        //     .and(() -> ground.getState()==GroundTake.State.SCORING||ground.getState()==GroundTake.State.HOLDING)
        //     .and(() -> arm.getState()!=Arm.State.INTAKING)
        //     .and(()-> driver.getRightStickButton())
        //         .onTrue(
        //             ground.intakeCoralSequence(() -> 1)
        //         );
            
        
        
        // driverC.y().onTrue(swerve.runOnce(swerve::toggleBrakes));

        // operator.povLeft().whileTrue(new TargetAlign(swerve));
        // new Trigger(() -> driver.getPOV() == 270).whileTrue(new TargetAlign(swerve, false));
        // new Trigger(() -> driver.getPOV() == 90).whileTrue(new TargetAlign(swerve, true));
        // Command rumbleD = Commands.startEnd(
        //     () -> driverC.setRumble(RumbleType.kBothRumble, .5),
        //     () -> driverC.setRumble(RumbleType.kBothRumble, 0)
        // ).until(() -> arm.getState() == Arm.State.SCORING);

        
        boolean testing = false;
        
        driverC.leftBumper().whileTrue(testing ? new TargetAlign(swerve, false, false, false, false) : new TargetAlign(swerve, false));
        driverC.rightBumper().whileTrue(testing ? new TargetAlign(swerve, true, false, false, false) : new TargetAlign(swerve, true));

        driverC.back().onTrue(swerve.runOnce(swerve::zeroHeading));

        operator.povDown().or(driverC.povDown()).and(() -> arm.isSafeToLift() || elevator.getState() == Elevator.State.L1).onTrue(elevator.setTargetOnly(Elevator.State.L0));
        // driverC.povRight().onTrue(elevator.setTargetOnly(Elevator.State.L1));
        
        // new Trigger(() -> (driver.getLeftX()!=0
        //     ||driver.getLeftY()!=0
        //     ||driver.getRightX()!=0)
        //     &&(elevator.getState()==Elevator.State.L1||elevator.getState()==Elevator.State.L0)
        // )
        //     .onTrue(elevator.setTargetOnly(Elevator.State.L0))
        //     .onFalse(elevator.setTargetOnly(Elevator.State.L1));
        

        operator.povRight().or(driverC.povLeft()).and(arm::isSafeToLift).onTrue(elevator.setTargetOnly(Elevator.State.L3)).onFalse(arm.setTargetOnly(Arm.State.SCORING));
        operator.povUp().or(driverC.povUp()).and(arm::isSafeToLift).onTrue(elevator.setTargetOnly(Elevator.State.L4)).onFalse(arm.setTargetOnly(Arm.State.SCORING));

        operator.a().or(driverC.a()).and(elevator::isSafeToIntake).onTrue(arm.setTargetOnly(Arm.State.INTAKING));
        operator.b().or(driverC.b()).onTrue(arm.setTargetOnly(Arm.State.SCORING));
        operator.x().or(driverC.x()).onTrue(arm.setTargetOnly(Arm.State.HOLDING));
        operator.y().or(driverC.y()).onTrue(arm.setTargetOnly(Arm.State.DUNKING));

        // operator.rightBumper().whileTrue(take.intakeCoral(() -> .7).andThen(Commands.runOnce(() -> arm.setTargetOnly(Arm.State.HOLDING).schedule())));
        // operator.leftBumper().whileTrue(take.outtakeCoral(.5));

        operator.leftStick().whileTrue(arm.rezero(() -> .075));
        // operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, .05).whileTrue(arm.adjust(() -> -operator.getRightY()).finallyDo(arm::stay));

        operator.back().onTrue(new AutoScore(
            new TargetAlign(swerve, false),
            elevator,
            arm,
            take
        ));
        operator.start().onTrue(new AutoScore(
            new TargetAlign(swerve, true),
            elevator,
            arm,
            take
        ));

        driverC.leftTrigger(.02).or(operator.leftTrigger(.02)).onTrue(Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.name)));

        // led testing
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
        elevator.setDefaultCommand(elevator.runRawFeedforward(() -> -tester.getLeftY()));

        // arm
        // tester.rightBumper().and(tester.a()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // tester.rightBumper().and(tester.b()).whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // tester.rightBumper().and(tester.x()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // tester.rightBumper().and(tester.y()).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        arm.setDefaultCommand(arm.runRawFeedforward(() -> -tester.getRightY()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().finallyDo(() -> {
            if (AutoBuilder.shouldFlip())
                swerve.setHeading(FlippingUtil.flipFieldRotation(swerve.getHeading()));
        });
    }
}
