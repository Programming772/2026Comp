// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.VecBuilder;
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


public class SwerveDriveSubsystem extends SubsystemBase {
  private static final SwerveModuleSubsystem [] modules = {
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
    ),

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
    )
  };

  //private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private RobotConfig config;
  
  // makes pigeon - note to do the calibration
  public final static Pigeon2 gyro = new Pigeon2(13);
  public static Field2d field = new Field2d();

  // timer for getting samples of paths
  public Timer pathTimer = new Timer();

  private boolean slowDown = false;
  
  // creates a pose estimator object to get the position of the robot relative to the field
  public static final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    Constants.SwerveConstants.driveKinematics,
    getRotation(),
    new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    },
    new Pose2d(),
    VecBuilder.fill(0.1, 0.1, 0.1),
    VecBuilder.fill(1, 1, 1)
  );


  public SwerveDriveSubsystem() {
    SmartDashboard.putData("Field", field);
    // improves CAN%
    gyro.optimizeBusUtilization(100, 10);

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
  }
  
  
  @Override
  public void periodic() {
    // updates the odometry to the value returned by Photon vision
    // Optional<EstimatedRobotPose> estimate = visionSubsystem.getEstimatedPose();
    // estimate.ifPresent(this::updateOdometry);

    // if there are no april tags in view, using onboard sensors to estimate position
    // if (estimate.isEmpty())
      updateOdometryRobotRelative();
    
    // updates the minifield on Shuffle board to the estimated position of the bot
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    // updates position of simulated robot on ShuffleBoard relative to the odometry
    SmartDashboard.putNumber("Pose X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }
  
  
  public void setModuleStates (SwerveModuleState[] states) {
    // tells each module to go to the prefered state (speeds and rotation)
    if (slowDown) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.1);
    } else {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.maxTeleSpeed);
    }

    // sets the state for all modules
    for (int i = 0; i < 4; i++)
      modules[i].setState(states[i]);
  }
  
  
  public void stopSwerveDrive() {
    // stops all motors in the swerve drive
    for (SwerveModuleSubsystem module : modules)
      module.stopMotors();
  }
  
  
  public static Rotation2d getRotation() {
    // returns the radian rotation of the gyro
    return new Rotation2d(Units.degreesToRadians((gyro.getYaw().getValueAsDouble()) % 360));
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


  public void autoPose(Pose2d newHeading) {
    poseEstimator.resetPose(newHeading);
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


  public void setSpeeds(ChassisSpeeds speeds) {
    // ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] states = Constants.SwerveConstants.driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.maxAutoSpeed);

    // sets the state for all modules
    for (int i = 0; i < 4; i++)
      modules[i].setState(states[i]);
  }


  public void slowSpeed() {
    slowDown = true;
  }


  public void regularSpeed() {
    slowDown = false;
  }
}
