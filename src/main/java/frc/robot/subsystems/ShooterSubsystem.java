// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX flywheel = new TalonFX(ShooterConstants.flywheelID);
  TalonFX hood = new TalonFX(ShooterConstants.hoodID);
  TalonFX turret = new TalonFX(ShooterConstants.turretID);
  TalonFX tower = new TalonFX(ShooterConstants.towerID);

  VelocityVoltage flywheelRequest = new VelocityVoltage(0);
  PositionVoltage hoodRequest = new PositionVoltage(0);
  PositionVoltage turretRequest = new PositionVoltage(0);
  VelocityVoltage towerRequest = new VelocityVoltage(0);
  
  public ShooterSubsystem() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 100;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0;
    flywheelConfig.Slot0.kP = 0;
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
    flywheelConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.StatorCurrentLimit = 50;
    hoodConfig.Slot0.kS = 0;
    hoodConfig.Slot0.kV = 0;
    hoodConfig.Slot0.kP = 0;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.Voltage.PeakForwardVoltage = 12.0;
    hoodConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.CurrentLimits.StatorCurrentLimit = 50;
    turretConfig.Slot0.kS = 0;
    turretConfig.Slot0.kV = 0;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretConfig.Voltage.PeakForwardVoltage = 12.0;
    turretConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration towerConfig = new TalonFXConfiguration();
    towerConfig.CurrentLimits.StatorCurrentLimit = 100;
    towerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    towerConfig.Slot0.kS = 0;
    towerConfig.Slot0.kV = 0;
    towerConfig.Slot0.kP = 0;
    towerConfig.Slot0.kI = 0;
    towerConfig.Slot0.kD = 0;
    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    towerConfig.Voltage.PeakForwardVoltage = 12.0;
    towerConfig.Voltage.PeakReverseVoltage = -12.0;

    // applies the configs to the motors
    flywheel.getConfigurator().apply(new TalonFXConfiguration());
    flywheel.getConfigurator().apply(flywheelConfig);
    hood.getConfigurator().apply(new TalonFXConfiguration());
    hood.getConfigurator().apply(hoodConfig);
    turret.getConfigurator().apply(new TalonFXConfiguration());
    turret.getConfigurator().apply(turretConfig);
    tower.getConfigurator().apply(new TalonFXConfiguration());
    tower.getConfigurator().apply(towerConfig);
  }

  @Override
  public void periodic() {
  }

  public void setflywheelRPM(double targetRPM) {
    flywheel.setControl(flywheelRequest.withVelocity(targetRPM / 60));
  }

  public void setTowerRPM(double targetRPM) {
    tower.setControl(towerRequest.withVelocity(targetRPM / 60));
  }

  public double getHoodPos() {
    return hood.getPosition().getValueAsDouble();
  }
}
