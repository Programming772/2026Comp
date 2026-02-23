// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Camera */
public class Camera {
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  
  public Camera(String cameraName, Transform3d cameraPos) {
    this.camera = new PhotonCamera(cameraName);
    this.poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos);
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public String getName() {
    return camera.getName();
  }

  public PhotonPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  public List<PhotonPipelineResult> getResults() {
    return camera.getAllUnreadResults();
  }
}
