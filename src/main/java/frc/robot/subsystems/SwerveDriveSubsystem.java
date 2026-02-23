// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/**
 * Represents a full Swerve Drive.
 * @author Koltin Scane
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private static final SwerveModuleSubsystem [] modules = {
    // makes a SwerveModuleSubsystem for the front left module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.FrontLeftConstants.frontLeftPropulsionID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftTurningID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftCANCoderID, 
      Constants.SwerveConstants.FrontLeftConstants.frontLeftCANCoderOffset, 
      false,
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the front right module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.FrontRightConstants.frontRightPropulsionID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightTurningID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightCANCoderID, 
      Constants.SwerveConstants.FrontRightConstants.frontRightCANCoderOffset, 
      false,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the back left module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.BackLeftConstants.backLeftPropulsionID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftTurningID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftCANCoderID, 
      Constants.SwerveConstants.BackLeftConstants.backLeftCANCoderOffset, 
      false,
      InvertedValue.Clockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    ),
    // makes a SwerveModuleSubsystem for the back right module
    new SwerveModuleSubsystem(
      Constants.SwerveConstants.BackRightConstants.backRightPropulsionID, 
      Constants.SwerveConstants.BackRightConstants.backRightTurningID, 
      Constants.SwerveConstants.BackRightConstants.backRightCANCoderID, 
      Constants.SwerveConstants.BackRightConstants.backRightCANCoderOffset, 
      false,
      InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive
    )
  };

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeonID);
  private final PIDController thetaPID = new PIDController(SwerveConstants.thetaPIDkp, SwerveConstants.thetaPIDki, SwerveConstants.thetaPIDkd);
  private RobotConfig config;
  private Field2d field = new Field2d();
  private Rotation2d targetRot = getRotation();
  private boolean slowDown = false;
  private boolean isHeadingLocked = true;
  
  // creates a pose estimator object to get the position of the robot relative to the field
  public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    Constants.SwerveConstants.driveKinematics,
    getRotation(),
    new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(10))
  );

  public SwerveDriveSubsystem() {
    SmartDashboard.putData("Field", field);

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println(e);
    }

    // instantiates the PathPlanner AutoBuilder 
    AutoBuilder.configure(
      this::getPose, 
      this::autoPose, 
      this::getRobotSpeeds, 
      this::setSpeeds, 
      new PPHolonomicDriveController(
        new PIDConstants(
          Constants.SwerveConstants.propulsionPIDkp, 
          Constants.SwerveConstants.propulsionPIDki, 
          Constants.SwerveConstants.propulsionPIDkd), 
        new PIDConstants(
          Constants.SwerveConstants.turningPIDkp, 
          Constants.SwerveConstants.turningPIDki, 
          Constants.SwerveConstants.turningPIDkd)
      ), 
      config, 
      () -> {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
      }, 
      this
    );

    gyro.optimizeBusUtilization(100, 10);
    thetaPID.enableContinuousInput(-Math.PI, Math.PI);
  }
  
  @Override
  public void periodic() {
    // pose estimation relative to robot sensors and vision
    updateOdometryRobotRelative();
    Optional<EstimatedRobotPose> visionPose = visionSubsystem.getEstimatedPose();

    if (visionPose.isPresent()) {
      EstimatedRobotPose estimate = visionPose.get();

      // Only trust multi-tag
      if (estimate.targetsUsed.size() >= 2) {
        poseEstimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
      }
    }
    
    // updates the minifield on Shuffle board to the estimated position of the bot
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    // updates position of simulated robot on ShuffleBoard relative to the odometry
    SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }
  
  /**
   * @return Rotation of the gyro.
   */
  public Rotation2d getRotation() {
    // returns 
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void resetHeading() {
    // resets the rotation of the gyro to 0
    gyro.reset();
  }

  public void updateOdometryRobotRelative() {
    // updates the odometry relative to the onboard sensors
    poseEstimator.update(
      getRotation(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      }
    );
  }

  public void autoPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      getRotation(),  // current gyro rotation
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      },
      newPose
    );
  }

  public void updateOdometry(EstimatedRobotPose pose) {
    // updates odometry relative to the returned position form Photon vision
    poseEstimator.resetPose(pose.estimatedPose.toPose2d());
    poseEstimator.resetRotation(getRotation());    
  }

  public Pose2d getPose() {
    // returns the pose that is provided by the odometry
    return poseEstimator.getEstimatedPosition();
  }
  
  public ChassisSpeeds getRobotSpeeds() {
    return Constants.SwerveConstants.driveKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState()
    };
  }

  public void generateSpeeds(double xSpeed, double ySpeed, double thetaSpeed) {
    if (Math.abs(thetaSpeed) < 0.05) {
      if (!isHeadingLocked) {
        targetRot = getRotation();
        thetaPID.reset();
        isHeadingLocked = true;
      }

      thetaSpeed = -thetaPID.calculate(getRotation().getRadians(), targetRot.getRadians());
    } else {
      isHeadingLocked = false;
      targetRot = getRotation();
    }

    thetaSpeed = MathUtil.clamp(thetaSpeed, -4.0, 4.0);
    
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, getRotation());

    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    
    setSpeeds(discretizedSpeeds);
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states =
      Constants.SwerveConstants.driveKinematics
        .toSwerveModuleStates(speeds);

    if (slowDown) {
      SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.SwerveConstants.maxVelocity * 0.3
      );
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.SwerveConstants.maxVelocity
      );
    }

    for (int i = 0; i < 4; i++) {
      modules[i].setState(states[i], speeds.omegaRadiansPerSecond);
    }
  }

  public void slowSpeed() {
    slowDown = true;
  }

  public void regularSpeed() {
    slowDown = false;
  }

  public void setTargetRot(Rotation2d angle) {
    targetRot = angle;
  }

  public void humpRot() {
    double currentAngle = getRotation().getDegrees();

    targetRot = Rotation2d.fromDegrees(45 + 90 * Math.floor(currentAngle / 90));
  }
}
