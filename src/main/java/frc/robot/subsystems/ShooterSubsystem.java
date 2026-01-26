// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // TalonFX topFlyWheel = new TalonFX(ShooterConstants.topFlyWheelID);
  // TalonFX bottomFlyWheel = new TalonFX(ShooterConstants.bottomFlyWheelID);
  // TalonFX hood = new TalonFX(ShooterConstants.hoodID);
  // TalonFX turret = new TalonFX(ShooterConstants.turretID);

  ProfiledPIDController hoodPID = new ProfiledPIDController(ShooterConstants.hoodPIDkp, ShooterConstants.hoodPIDki, ShooterConstants.hoodPIDkd, new TrapezoidProfile.Constraints(1, 0.5));
  ProfiledPIDController turretPID = new ProfiledPIDController(ShooterConstants.turretPIDkp, ShooterConstants.turretPIDki, ShooterConstants.turretPIDkd, new TrapezoidProfile.Constraints(1, 0.5));
  
  SimpleMotorFeedforward hoodFF = new SimpleMotorFeedforward(ShooterConstants.hoodFFks, ShooterConstants.hoodFFkv, ShooterConstants.hoodFFka);
  SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(ShooterConstants.turretFFks, ShooterConstants.turretFFkv, ShooterConstants.turretFFka);
  
  public ShooterSubsystem() {
    hoodPID.reset(0);
    turretPID.reset(0);
  }

  @Override
  public void periodic() {
    // hood.setVoltage(hoodPID.calculate(getHoodPos()) + turretFF.calculate(hoodPID.getSetpoint().velocity));
  }

  // public double getHoodPos() {
  //   return hood.getPosition().getValueAsDouble();
  // }

  public void setHoodGoal(double goal) {
    hoodPID.setGoal(goal);
  }

  public void setTurretGoal (double goal) {
    turretPID.setGoal(goal);
  }
}
