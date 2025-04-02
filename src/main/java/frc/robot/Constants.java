// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.util.COTSTalonFXSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double period = .015; // seconds
    public static final double brownoutVoltage = 6.0;

    public static final class OperatorConstants {
        public static final int controllerPort = 1;
        public static final double stickDeadband = .02;
    }

    public static final class DriverConstants {
        public static final int controllerPort = 0;
        public static final double stickDeadband = .01;

        public static final double swerveMaxTransSpeed = 0.50;
        public static final double swerveMaxTurnSpeed = 0.50;
        public static final double swerveSlowSpeed = 0.20;

        public static final TrapezoidProfile.Constraints tipConstraints = new TrapezoidProfile.Constraints(25.0, 100.0);
    }

    public static final class LimelightConstants {
        public static final String name = "limelight";
        public static final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public static final class IntakeConstants {
        public static final SparkBaseConfig motorConfig = new SparkFlexConfig()
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false)
                .smartCurrentLimit(85, 80);
    }

    public static final class ElevatorConstants {
        public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(35)
                .withStatorCurrentLimit(30))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));

        public static ElevatorFeedforward feedforward = new ElevatorFeedforward(0.12, 0.42, 0, 0);
    }

    public static final class ArmConstants {
        public static final TalonFXConfiguration wristConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimit(30))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));

        public static final Angle START_HORIZONTAL_OFFSET = Degrees.of(116.43310546875); // cad measured is ~116.852
        
        public static final ArmFeedforward feedforward = new ArmFeedforward(0, 0.141, 0, 0);
    }
    public static final class GroundConstants {
        public static final TalonFXConfiguration wristConfig = new TalonFXConfiguration()
            .withCurrentLimits( new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimit(30))
            .withMotorOutput( new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));

        public static final Angle START_HORIZONTAL_OFFSET = Degrees.of(-1); 

        public static final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
    }

    public static final class SwerveConstants {
        public static final COTSTalonFXSwerveConstants swerveType = COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        public static final TalonFXConfiguration swerveSteerFXConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        public static final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.50); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.50); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = swerveType.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );
        
        public static final RobotConfig autoPathFollowerConfig = new RobotConfig(// = RobotConfig.fromGUISettings();
            51.26,
            6.3,
            new ModuleConfig(
                Units.inchesToMeters(2.5),
                5.45,
                1.2,
                // new DCMotor(
                //     12.,
                //     4.69 * swerveType.driveGearRatio,
                //     257,
                //     1.5,
                //     668.112038 / swerveType.driveGearRatio,
                //     1
                // ),
                DCMotor.getFalcon500(1).withReduction(swerveType.driveGearRatio),
                60,//swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit,
                1
            ),
            kinematics.getModules() // Default path replanning config. See the API for the options here
        );
        public static final PPHolonomicDriveController autoFollowerController = new PPHolonomicDriveController(
            new PIDConstants(5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
        );

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot


        public static final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.32, 1.51, 0.27);

        // order: fl, fr, bl, br
        public static final ModuleConstants[] moduleConstants = new ModuleConstants[] {
            new ModuleConstants(1, 8, 10, Rotation2d.fromDegrees(-30.498046875)),
            new ModuleConstants(7, 6, 11, Rotation2d.fromDegrees(28.652343749999996)),
            new ModuleConstants(5, 4, 12, Rotation2d.fromDegrees(-118.30078125)),
            new ModuleConstants(3, 2, 13, Rotation2d.fromDegrees(-161.71875))
        };

        public static final class ModuleConstants {
            public final int driveMotorID;
            public final int steerMotorID;
            public final int absoluteEncoderID;
            public final Rotation2d angleOffset;

            public ModuleConstants(int driveMotorID, int steerMotorID, int absoluteEncoderID, Rotation2d angleOffset) {
                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.absoluteEncoderID = absoluteEncoderID;
                this.angleOffset = angleOffset;
            }
        }
        
        
        static {
            // pathplanner config
            // try {
            //     autoPathFollowerConfig = RobotConfig.fromGUISettings();
            // } catch (IOException e) {
            //     // TODO Auto-generated catch block
            //     e.printStackTrace();
            //     // autoPathFollowerConfig = new RobotConfig(
            //     //     -1,
            //     //     -1,
            //     //     null,
            //     //     null
            //     // );
            // } catch (ParseException e) {
            //     // TODO Auto-generated catch block
            //     e.printStackTrace();
            //     // autoPathFollowerConfig = new RobotConfig(
            //     //     -1,
            //     //     -1,
            //     //     null,
            //     //     null
            //     // );
            // } finally {
            //     // autoPathFollowerConfig = new RobotConfig(
            //     //     -1,
            //     //     -1,
            //     //     null,
            //     //     null
            //     // );
            // }


            /* Swerve CANCoder Configuration */
            swerveCANcoderConfig.MagnetSensor.SensorDirection = swerveType.cancoderInvert;

            /* Swerve Angle Motor Configurations */
            /* Motor Inverts and Neutral Mode */
            swerveSteerFXConfig.MotorOutput.Inverted = swerveType.angleMotorInvert;
            swerveSteerFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            /* Gear Ratio and Wrapping Config */
            swerveSteerFXConfig.Feedback.SensorToMechanismRatio = swerveType.angleGearRatio;
            swerveSteerFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
            
            /* Current Limiting */
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            /* PID Config */
            swerveSteerFXConfig.Slot0.kP = swerveType.angleKP * .20;
            swerveSteerFXConfig.Slot0.kI = swerveType.angleKI;
            swerveSteerFXConfig.Slot0.kD = swerveType.angleKD;

            /* Swerve Drive Motor Configuration */
            /* Motor Inverts and Neutral Mode */
            swerveDriveFXConfig.MotorOutput.Inverted = swerveType.driveMotorInvert;
            swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            /* Gear Ratio Config */
            swerveDriveFXConfig.Feedback.SensorToMechanismRatio = swerveType.driveGearRatio;

            /* Current Limiting */
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =true;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            /* PID Config */
            swerveDriveFXConfig.Slot0.kP = 2.5;
            swerveDriveFXConfig.Slot0.kI = swerveType.angleKI;
            swerveDriveFXConfig.Slot0.kD = swerveType.angleKD;

            /* Open and Closed Loop Ramping */
            swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
            swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

            swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
            swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        }
    }
}
