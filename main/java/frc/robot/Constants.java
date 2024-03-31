// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants
  {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = (6.75 / 1.0);
    public static final double kAngleMotorGearRatio = (12.8 / 1.0);
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kAngleEncoderRot2Radians = kAngleMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter /  60;
    public static final double kAngleEncoderRPM2RadPerSec = kAngleEncoderRot2Radians / 60;
    public static final double kPTurning = 0.5; //some of these values to be changed and ask zaan

    }
  public static final class driveConstants
  {
    public static final double kTrackWidth = Units.inchesToMeters(27);
    public static final double kWheelBase = Units.inchesToMeters(24);
    public static final SwerveDriveKinematics kSwerveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), //translation 2d locates the swerve module in cords
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));
    
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kFrontRightDriveMotorPort = 13;
    public static final int kBackRightDriveMotorPort = 11;

    public static final int kFrontLeftAngleMotorPort = 16;
    public static final int kBackLeftAngleMotorPort = 6;
    public static final int kFrontRightAngleMotorPort = 12;
    public static final int kBackRightAngleMotorPort = 10;

    public static final int kFrontLeftAbsoluteAngleEncoderPort = 22;
    public static final int kBackLeftAbsoluteAngleEncoderPort = 23;
    public static final int kFrontRightAbsoluteAngleEncoderPort = 24;
    public static final int kBackRightAbsoluteAngleEncoderPort = 25;

    public static final double kMaxSpeedMetersPerSecond = 4;

    public static boolean kFrontLeftDriveEncoderReversed = false;
    public static boolean kBackLeftDriveEncoderReversed = false;
    public static boolean kFrontRightDriveEncoderReversed = false;
    public static boolean kBackRightDriveEncoderReversed = false; 

    public static boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static boolean kBackRightDriveAbsoluteEncoderReversed = false; 

    public static boolean kFrontLeftAngleEncoderReversed = false;
    public static boolean kBackLeftAngleEncoderReversed = false;
    public static boolean kFrontRightAngleEncoderReversed = false;
    public static boolean kBackRightAngleEncoderReversed = false; 


    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.79504393;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.86555523;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.13737665;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 6.22489131;

    // speed values
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDeadband = 0.05;


  }
}
