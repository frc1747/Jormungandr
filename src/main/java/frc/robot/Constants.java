// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.nio.file.attribute.PosixFileAttributeView;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Robot {
    public static NetworkTableInstance NT_INSTANCE;
  }

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double STICK_DEADBAND = 0.05;// was .1
  }

  public static class IntakeConstants {
    public static final int ROLLER_ONE = 41;
    public static final int ROLLER_TWO = 42;
    public static final int HINGE = 43;
    public static final int LIMIT_SWITCH = 1;

    public static final double STOWED = 0.0;
    public static final double DROPPED = 34; // was 64000
    public static final double CLEAN = 10;
    public static final double SLOW_POSITION = 10;

    public static final double IN_SPEED = 0.5; // was 1.00
    public static final double OUT_SPEED = -0.4; // was -0.75

    public static final double ROLLER_SPEED = 0.80; // was .9
    public static final double ROLLER_SPEED_CLEAN = 1.0; 
    public static final double PIVOT_IN_SPEED = 0.6;
    public static final double PIVOT_IN_SPEED_LOW = 0.3;
    public static final double PIVOT_OUT_SPEED = 0.6;
    public static final double IN_SLOW_FACTOR = 0.50;
    public static final double ROLLER_SPEED_ADJUST_NOTE = 0.20; // was 0.15

    public static final double UP_LIMIT = STOWED;
    public static final double DOWN_LIMIT = DROPPED;

    public static final int NOTE_LIMIT_SWITCH = 0;
  }

  public static class ShooterConstants {
    public static final int FRONT = 52;
    public static final int FRONT_TWO = 51;// left shooter
    public static final int HINGE = 54;
    public static final int LIMIT_SWITCH = 0;

    public static final double STOWED = 0;
    public static final double AMP = 97500*4;
    public static final double PODIUM = 68000;

    public static final double SHOOT_SPEED = 1;
    
    public static final double HINGE_SPEED = 1;

    public static final double FLYWHEEL_HIGH_SPEED = -16500;

    public static final double DOWN_LIMIT = STOWED;
    public static final double UP_LIMIT = 100000*4;
  }

  public static class FeederConstants {
      public static final int BACK = 53;
      public static final double TRANSITION_SPEED = 0.70;
      public static final double ADJUST_NOTE_SPEED = 0.40;
      public static final int LIMIT_SWITCH = 1;
      public static final int ADJUST_NOTE_MILLIS = 50;
  }

  public static class ClimberConstants {
    public static final int LEFT = 61;
    public static final int RIGHT = 62;

    public static final double CLIMBER_SPEED = 0.75;
    public static final double UP_LIMIT = 275000;

    public static final double SLOW_LIMIT = 50000;
  }
  
  public static class DrivetrainConstants {
    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    public static final SensorDirectionValue canCoderDirection = chosenModule.canCoderDirection; // Should the cancoder be inverted based on the swerve module we're using
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- (DO NOT USE, ENABLES ROBOT-CENTRIC)
    public static final int PIGEON_ID = 50;

    // Physical measurements
    public static final double trackWidth = Units.inchesToMeters(22.5);
    public static final double wheelBase = Units.inchesToMeters(22.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    public static final double CENTER_TO_WHEEL = Math.sqrt(Math.pow(wheelBase / 2.0, 2) + Math.pow(trackWidth / 2.0, 2));

    // Module gear ratios based on the swerve module we're using
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // Motor inverts based on the swerve module we're using
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    // Drive Motor Conversion Factors
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    // Swerve Kinematics 
    // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    // Swerve Current Limiting
    public static final int angleContinuousCurrentLimit = 40;
    public static final int anglePeakCurrentLimit = 100;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 40;
    public static final int drivePeakCurrentLimit = 100;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    // Swerve Voltage Compensation (default)
    public static final double voltageComp = 12.0;

    // Neutral Modes
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    // TODO: Tune these later
    public static final double MAX_SPEED = 4.1;  // Max speed in m/s
    public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s
    public static final double maxAngularVelocity = 10.0;  // Rad/s

    // TODO: Tune these later
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    public static final double DRIVE_KS = 0.0; 
    public static final double DRIVE_KV = 0.0;
    public static final double DRIVE_KA = 0.0;

    public static final double ANGLE_KP = chosenModule.angleKP;
    public static final double ANGLE_KI = chosenModule.angleKI;
    public static final double ANGLE_KD = chosenModule.angleKD;
    public static final double ANGLE_KF = chosenModule.angleKF;

    // Module Specific Constants
    // Front Left Module
    public static final class FRONT_LEFT { 
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(294.4); // was 297.9
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Front Right Module
    public static final class FRONT_RIGHT { 
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(336.9); // was 335.8
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    // Back Left Module
    public static final class REAR_LEFT { 
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(91.8); // was 266.4 - 180
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    // Back Right Module
    public static final class REAR_RIGHT { 
      public static final int driveMotorID = 31; 
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(273.7); // was 95.3 - 180
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static class VisionConstants {
    // Distance to move robot in AlignWithAprTag
    public static final double alignDistance = 0.165;
    public static final List<Pose2d> poseList = listMaker();
    
    private static List<Pose2d> listMaker(){
      List<Pose2d> poseList = new ArrayList<Pose2d>();
      
      poseList.add(new Pose2d(15.11, 3.82, new Rotation2d(0.0))); // April Tag 7, Left
      poseList.add(new Pose2d(15.10, 4.14, new Rotation2d(0.0))); // April Tag 7, Right
      // poseList.add(new Pose2d()); // April Tag 8, Left
      // poseList.add(new Pose2d()); // April Tag 8, Right
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());
      // poseList.add(new Pose2d());

      return poseList;
    }

    public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
    public static final class LimelightConstants {
      public static final double MIN_TAG_DIST_TO_BE_FAR = 3.0;

	public static String LIMELIGHT_NT_NAME;

      public static double LL_FORWARD;
      public static double LL_RIGHT;
      public static double LL_UP;
      public static double LL_ROLL;
      public static double LL_PITCH;
      public static double LL_YAW;

      public static double LL_MAX_TAG_CLEAR_DIST;
    }

       
  }


  public static class AutoConstants {
    
    /*
    public static final PathFollowerConfig pathFollowerConfig = 
      new PathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(1, 0.0, 0.01), // Rotation PID constants
        4.5, // Max module speed, in m/s
        Constants.DrivetrainConstants.CENTER_TO_WHEEL, //Math.sqrt(2)*Units.inchesToMeters(11.25), // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(false, false) 
      );*/
  } 
}
