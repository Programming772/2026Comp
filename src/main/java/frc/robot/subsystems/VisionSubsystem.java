// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Camera;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {  
  // objects instantiation for vision logic
  public Camera[] cameras = new Camera[Constants.cameraNames.length];

  private Map<String, List<PhotonPipelineResult>> results = new HashMap<String, List<PhotonPipelineResult>>();
  private Map<String, List<PhotonTrackedTarget>> aprilTags;
  
  public VisionSubsystem() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new Camera(Constants.cameraNames[i], Constants.cameraPositions[i]);
    } 
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("getName()", true);
    Map<String, List<PhotonPipelineResult>> newResults = new HashMap<String, List<PhotonPipelineResult>>();
    
    // retrieves all new results for each camera on the robot
    for (PhotonCamera camera : cameras) {
      newResults.put(camera.getName(), camera.getAllUnreadResults());

      if (! (newResults.isEmpty())) {
        results = newResults;
      }
    }

    if (aprilTags != null) {
      for (int i = 0; i < aprilTags.size(); i++) {
        double[] tagIDs = new double[aprilTags.get(Constants.cameraNames[i]).size()];

        // gets the ID of each seen April tag
        for (int k = 0; k < tagIDs.length; k++) {
          tagIDs[k] = aprilTags.get(Constants.cameraNames[i]).get(k).getFiducialId();
        }

        // displays the seen april tags on Shuffleboard
        SmartDashboard.putString("April Tags", Arrays.toString(tagIDs));
      }
    }
  }

  
  public Map<String, List<PhotonTrackedTarget>> getAprilTags() {
    aprilTags = new HashMap<String, List<PhotonTrackedTarget>>();

    // gets each seen april tag from the results
    if (results.size() != 0) {
      for (int i = 0; i < results.size(); i++) {
        for (PhotonPipelineResult result : results.get(Constants.cameraNames[i])) {
          List<PhotonTrackedTarget> newtargets = result.getTargets();
          
          aprilTags.put(Constants.cameraNames[i], newtargets);
        }
      }
    }

    // returns an array of seen april tags
    return aprilTags;
  }


  public Optional<EstimatedRobotPose> getEstimatedPose() {
    Optional<EstimatedRobotPose> estimatedRobot = Optional.empty();

    for (int i = 0; i < results.size(); i++) {
      // updates the estimated relative to the multi tag result
      for (PhotonPipelineResult multiTag : results.get(Constants.cameraNames[i])) {
        estimatedRobot = poseEstimators[0].update(multiTag);
      }
    }

    // returns the estimated robot position
    return estimatedRobot;
  }
}