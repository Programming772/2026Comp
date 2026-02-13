// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ShooterConstants;

// public class ShooterSubsystem extends SubsystemBase {
//   TalonFX topFlyWheel = new TalonFX(ShooterConstants.topFlyWheelID);
//   TalonFX bottomFlyWheel = new TalonFX(ShooterConstants.bottomFlyWheelID);
//   TalonFX hood = new TalonFX(ShooterConstants.hoodID);
//   TalonFX turret = new TalonFX(ShooterConstants.turretID);

  
//   public ShooterSubsystem() {
//     TalonFXConfiguration turretConfig = new TalonFXConfiguration();
//     turretConfig.CurrentLimits.StatorCurrentLimit = 50;
//     turretConfig.CurrentLimits.SupplyCurrentLimit = 70;
//   }

//   @Override
//   public void periodic() {
//   }

//   public double getHoodPos() {
//     return hood.getPosition().getValueAsDouble();
//   }
// }
