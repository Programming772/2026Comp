// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = new CANBus("CANivore");

  TalonFX flywheel1 = new TalonFX(ShooterConstants.flywheel1ID);
  TalonFX flywheel2 = new TalonFX(ShooterConstants.flywheel2ID);
  TalonFX tower = new TalonFX(ShooterConstants.towerID, kCANBus);

  public ShooterSubsystem() {
    //turret.setPosition(0);
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 100;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0.1275;
    flywheelConfig.Slot0.kP = 0.367;
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
    flywheelConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration towerConfig = new TalonFXConfiguration();
    towerConfig.CurrentLimits.StatorCurrentLimit = 100;
    towerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0;
    flywheelConfig.Slot0.kP = 0;
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    towerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    towerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    towerConfig.Voltage.PeakForwardVoltage = 12.0;
    towerConfig.Voltage.PeakReverseVoltage = -12.0;

    // applies the configs to the motors
    tower.getConfigurator().apply(new TalonFXConfiguration());
    tower.getConfigurator().apply(towerConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel1 RPM", getFlywheel1RPM());
    SmartDashboard.putNumber("Flywheel2 RPM", getFlywheel2RPM());
    SmartDashboard.putNumber("Tower RPM", getTowerRPM());
  }

  public void setflywheel1RPM(double targetRPM) {
    flywheel1.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getFlywheel1RPM() {
    return flywheel1.getVelocity().getValueAsDouble() * 60;
  }

  public void manualFlywheel1(double speed) {
    flywheel1.set(speed);
  }

  public void setflywheel2RPM(double targetRPM) {
    flywheel2.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getFlywheel2RPM() {
    return flywheel2.getVelocity().getValueAsDouble() * 60;
  }

  public void manualFlywheel2(double speed) {
    flywheel2.set(speed);
  }

  public void setTowerRPM(double targetRPM) {
    tower.setControl(new VelocityVoltage(targetRPM / 60).withSlot(0));
  }

  public double getTowerRPM() {
    return tower.getVelocity().getValueAsDouble() * 60;
  }

  public void manualTower(double speed) {
    tower.set(speed);
  }
}
