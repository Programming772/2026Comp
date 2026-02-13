// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class TeleoperatedControlCommand extends Command {
  private DoubleSupplier xSpeedSupplier, ySpeedSupplier, thetaSpeedSupplier;
  private BooleanSupplier fieldOrientedSupplier;
  private SwerveDriveSubsystem swerveSubsystem;
  private double deadband = 0.15;

  public TeleoperatedControlCommand(SwerveDriveSubsystem swerveDriveSubsystem, BooleanSupplier fieldOrientedSupplier, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier thetaSpeedSupplier) {
    this.swerveSubsystem = swerveDriveSubsystem;
    this.fieldOrientedSupplier = fieldOrientedSupplier;
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;
    this.thetaSpeedSupplier = thetaSpeedSupplier;
    
    addRequirements(swerveSubsystem);
  }

  
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets the absolute values of the speeds, sets them to 0 if too slow
    double xSpeed = Math.abs(xSpeedSupplier.getAsDouble()) > deadband ? xSpeedSupplier.getAsDouble() : 0.0;
    double ySpeed = Math.abs(ySpeedSupplier.getAsDouble()) > deadband ? ySpeedSupplier.getAsDouble() : 0.0;
    double thetaSpeed = Math.abs(thetaSpeedSupplier.getAsDouble()) > deadband ? thetaSpeedSupplier.getAsDouble() : 0.0;

    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
    thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);


    // gets the speeds from the joystick inputs
    xSpeed *= Constants.SwerveConstants.maxVelocity;
    ySpeed *= Constants.SwerveConstants.maxVelocity;
    thetaSpeed *= Constants.SwerveConstants.maxAngularVelocity;

    ChassisSpeeds chassisSpeeds;

    // creates speeds relative to the wanted robot orientation
    if (fieldOrientedSupplier.getAsBoolean()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, swerveSubsystem.getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }

    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    // turns the chassis speeds into modules using the kinematics of the swerve drive
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.driveKinematics.toSwerveModuleStates(discretizedSpeeds);
    
    // applies the swerve module states to the swerve drive
    swerveSubsystem.setModuleStates(moduleStates, discretizedSpeeds.omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
