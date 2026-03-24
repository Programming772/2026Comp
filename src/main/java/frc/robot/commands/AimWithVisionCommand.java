// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimWithVisionCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private VisionSubsystem visionSubsystem;

  List<Integer> useableTags = new ArrayList<>();

  public AimWithVisionCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(shooterSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
      useableTags.addAll(Arrays.asList(2, 3, 4, 5, 8, 9, 10, 11));
    } else {
      useableTags.addAll(Arrays.asList(18, 19, 20, 21, 24, 25, 26, 27));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(PhotonTrackedTarget aprilTag : visionSubsystem.getAprilTags().get(Constants.cameraNames[0])) {
      if (useableTags.contains(aprilTag.fiducialId)) {
        if (aprilTag.fiducialId == 10 || aprilTag.fiducialId == 26) {
          // subtraction here assumes CW position for turret rot might change
          shooterSubsystem.setTurretPosition(Rotation2d.fromDegrees(shooterSubsystem.getTurretAngle().getDegrees() - aprilTag.getYaw()));
          // find linear relation of distance to hub and hood angle
          shooterSubsystem.setHoodPosition((AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTagPose(aprilTag.getFiducialId()).get().getZ() - Constants.cameraPositions[0].getZ()) / aprilTag.getPitch());
          return;
        }
      }
    }

    shooterSubsystem.setTurretPosition(Rotation2d.fromDegrees(shooterSubsystem.getTurretAngle().getDegrees() + 10));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
