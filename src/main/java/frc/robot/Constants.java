// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double stickDeadband = .02;
    }

    public static final class DriverConstants {
        public static final double swerveMaxSpeed = 0.5;
        public static final double swerveSlowSpeed = 0.2;
    }

    public static final class ElevatorConstants {
        public static final int[] motorIDs = new int[] {-1, -1};

        public static final PIDController pidConroller = new PIDController(0, 0, 0);
        public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);
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
        
        public static RobotConfig autoPathFollowerConfig;// = RobotConfig.fromGUISettings();
        //     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //     new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
        //     4.5,//Constants.Swerve.maxSpeed, // Max module speed, in m/s
        //     0.4,//new Translation2d(Constants.Swerve.trackWidth, Constants.Swerve.wheelBase).getNorm() / 2, // Drive base radius in meters. Distance from robot center to furthest module.
        //     new ReplanningConfig() // Default path replanning config. See the API for the options here
        // );
        public static final PPHolonomicDriveController autoFollowerController = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        );

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot


        public static final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.32, 1.51, 0.27);

        // order: fl, fr, bl, br
        public static final ModuleConstants[] moduleConstants = new ModuleConstants[]{
            new ModuleConstants(1, 8, 10, Rotation2d.fromDegrees(-30.498046875)),
            new ModuleConstants(7, 6, 11, Rotation2d.fromDegrees(28.652343749999996)),
            new ModuleConstants(3, 4, 12, Rotation2d.fromDegrees(-118.30078125)),
            new ModuleConstants(5, 2, 13, Rotation2d.fromDegrees(-25.48828125))
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
            try {
                autoPathFollowerConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                // autoPathFollowerConfig = new RobotConfig(
                //     -1,
                //     -1,
                //     null,
                //     null
                // );
            } catch (ParseException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                // autoPathFollowerConfig = new RobotConfig(
                //     -1,
                //     -1,
                //     null,
                //     null
                // );
            } finally {
                // autoPathFollowerConfig = new RobotConfig(
                //     -1,
                //     -1,
                //     null,
                //     null
                // );
            }


            /** Swerve CANCoder Configuration */
            swerveCANcoderConfig.MagnetSensor.SensorDirection = swerveType.cancoderInvert;

            /** Swerve Angle Motor Configurations */
            /* Motor Inverts and Neutral Mode */
            swerveSteerFXConfig.MotorOutput.Inverted = swerveType.angleMotorInvert;
            swerveSteerFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            /* Gear Ratio and Wrapping Config */
            swerveSteerFXConfig.Feedback.SensorToMechanismRatio = swerveType.angleGearRatio;
            swerveSteerFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
            
            /* Current Limiting */
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
            swerveSteerFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            /* PID Config */
            swerveSteerFXConfig.Slot0.kP = swerveType.angleKP;
            swerveSteerFXConfig.Slot0.kI = swerveType.angleKI;
            swerveSteerFXConfig.Slot0.kD = swerveType.angleKD;

            /** Swerve Drive Motor Configuration */
            /* Motor Inverts and Neutral Mode */
            swerveDriveFXConfig.MotorOutput.Inverted = swerveType.driveMotorInvert;
            swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            /* Gear Ratio Config */
            swerveDriveFXConfig.Feedback.SensorToMechanismRatio = swerveType.driveGearRatio;

            /* Current Limiting */
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =true;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = 60;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            /* PID Config */
            swerveDriveFXConfig.Slot0.kP = swerveType.angleKP;
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
