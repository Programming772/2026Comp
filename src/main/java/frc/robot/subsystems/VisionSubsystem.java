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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Map<String, List<PhotonPipelineResult>> newResults = new HashMap<String, List<PhotonPipelineResult>>();
    
    // retrieves all new results for each camera on the robot
    for (Camera camera : cameras) {
      newResults.put(camera.getName(), camera.getResults());
    }

    if (!(newResults.isEmpty())) {
      results = newResults;
    }

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

  public Map<String, List<PhotonTrackedTarget>> getAprilTags() {
    Map<String, List<PhotonTrackedTarget>> aprilTags = new HashMap<>();

    for (Camera camera : cameras) {
      List<PhotonPipelineResult> cameraResults = results.get(camera.getName());
      if (cameraResults == null) continue;

      for (PhotonPipelineResult result : cameraResults) {
        aprilTags.put(camera.getName(), result.getTargets());
      }
    }

    return aprilTags;
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    for (Camera camera : cameras) {
    List<PhotonPipelineResult> cameraResults = results.get(camera.getName());
    if (cameraResults == null) continue;

    for (PhotonPipelineResult multiTag : cameraResults) {
      Optional<EstimatedRobotPose> pose = camera.getPoseEstimator().update(multiTag);
      
      if (pose.isPresent()) {
        return pose; // return first valid pose
      }
    }
  }

  return Optional.empty();
  }
}