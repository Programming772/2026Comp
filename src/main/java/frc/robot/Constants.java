// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;


public final class Constants {
  // array of all connected cameras
  public static final String[] cameraNames = {"ArduCam1", "ArduCam2"};

  public static final double leftPoleYaw = 0;
  public static final double rightPoleYaw = 0;

  // object for trajectory generation configs
  public static final TrajectoryConfig pathConfig = new TrajectoryConfig(3, 1);


  public final class SwerveConstants {
    public static final double maxVelocity = 4;
    public static final double maxAcceleration = 1;
    public static final double maxAngularVelocity = 2 * Math.PI;
    public static final double maxAngularAcceleration = 4 * Math.PI;

    // physical constants of the swerve drive modules
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);
    public static final double kDriveGearRatio = 5.9;
    public static final double kSteerGearRatio = 12.8;

    // constants for front left module
    public static final class FrontLeftConstants {
      public static final int frontLeftPropulsionID = 1;
      public static final int frontLeftTurningID = 2;
      public static final int frontLeftCANCoderID = 3;
      public static final Angle frontLeftCANCoderOffset = Rotations.of(0);
    }

    // constants for front right module
    public static final class FrontRightConstants {
      public static final int frontRightPropulsionID = 4;
      public static final int frontRightTurningID = 5;
      public static final int frontRightCANCoderID = 6;
      public static final Angle frontRightCANCoderOffset = Rotations.of(-0.3515625);
    }

    // constants for back left module
    public static final class BackLeftConstants {
      public static final int backLeftPropulsionID = 7;
      public static final int backLeftTurningID = 8;
      public static final int backLeftCANCoderID = 9;
      public static final Angle backLeftCANCoderOffset = Rotations.of(1.142578125);
    }

    // constants for back right module
    public static final class BackRightConstants {
      public static final int backRightPropulsionID = 10;
      public static final int backRightTurningID = 11;
      public static final int backRightCANCoderID = 12;
      public static final Angle backRightCANCoderOffset = Rotations.of(0.52734375);
    }

    // propulsion PID coefficients
    public static final double propulsionPIDkp = 0.0;
    public static final double propulsionPIDki = 0.0;
    public static final double propulsionPIDkd = 0.0;

    // propulsion FF coefficients
    public static final double propulsionFFka = 0.0;
    public static final double propulsionFFks = 0.3;
    public static final double propulsionFFkv = 6;

    // turning PID coefficients
    public static final double turningPIDkp = 3;
    public static final double turningPIDki = 0.0;
    public static final double turningPIDkd = 0;
    
    // turning FF coefficients
    public static final double turningFFka = 0.0;
    public static final double turningFFks = 0.0;
    public static final double turningFFkv = 0.0;

    // distance between modules
    public static final double kTrackWidth = Units.inchesToMeters(23);
    public static final double kWheelBase  = Units.inchesToMeters(23);

    // creates a SwerveDriveKinematics object using the locations of the modules relative to centre of the robot
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics (
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // front left module
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // front right module
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // back left module
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // back right module
    );
  }


  public final class ShooterConstants {
    // topFlyWheel, bottomFlyWheel, hood, turret IDs
    public static final int topFlyWheelID = 0;
    public static final int bottomFlyWheelID = 0;
    public static final int hoodID = 0;
    public static final int turretID = 0;

    public static final double hoodPIDkp = 0.0;
    public static final double hoodPIDki = 0.0;
    public static final double hoodPIDkd = 0.0;

    public static final double hoodFFka = 0.0;
    public static final double hoodFFks = 0.0;
    public static final double hoodFFkv = 0.0;

    public static final double turretPIDkp = 0.0;
    public static final double turretPIDki = 0.0;
    public static final double turretPIDkd = 0.0;

    public static final double turretFFka = 0.0;
    public static final double turretFFks = 0.0;
    public static final double turretFFkv = 0.0;
  }
}
