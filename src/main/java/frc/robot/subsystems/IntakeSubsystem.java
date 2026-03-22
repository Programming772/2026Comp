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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX feeder = new TalonFX(IntakeConstants.feederID);
  TalonFX intakeArm = new TalonFX(IntakeConstants.intakeArmID);
  TalonFX intakeRoller = new TalonFX(IntakeConstants.intakeRollerID);

  VelocityVoltage feederRPM = new VelocityVoltage(0);
  PositionVoltage intakeArmPosition = new PositionVoltage(0);
  VelocityVoltage intakeRollerRPM = new VelocityVoltage(0);
  
  public IntakeSubsystem() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = 100;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
    feederConfig.Slot0.kS = 0;
    feederConfig.Slot0.kV = 0;
    feederConfig.Slot0.kP = 0;
    feederConfig.Slot0.kI = 0;
    feederConfig.Slot0.kD = 0;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feederConfig.Voltage.PeakForwardVoltage = 12.0;
    feederConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();
    intakeArmConfig.CurrentLimits.StatorCurrentLimit = 50;
    intakeArmConfig.Slot0.kS = 0;
    intakeArmConfig.Slot0.kV = 0;
    intakeArmConfig.Slot0.kP = 0;
    intakeArmConfig.Slot0.kI = 0;
    intakeArmConfig.Slot0.kD = 0;
    intakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeArmConfig.Voltage.PeakForwardVoltage = 12.0;
    intakeArmConfig.Voltage.PeakReverseVoltage = -12.0;
    
    TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
    intakeRollerConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeRollerConfig.Slot0.kS = 0;
    intakeRollerConfig.Slot0.kV = 0;
    intakeRollerConfig.Slot0.kP = 0;
    intakeRollerConfig.Slot0.kI = 0;
    intakeRollerConfig.Slot0.kD = 0;
    intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeRollerConfig.Voltage.PeakForwardVoltage = 12.0;
    intakeRollerConfig.Voltage.PeakReverseVoltage = -12.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder RPM", getFeederRPM());
    SmartDashboard.putNumber("Intake Pos", getIntakePosition());
    SmartDashboard.putNumber("Intake RPM", getIntakeRollerRPM());
  }

  public void setFeederRPM(double targetRPM) {
    feeder.setControl(feederRPM.withVelocity(targetRPM / 60));
  }

  public double getFeederRPM() {
    return feeder.getVelocity().getValueAsDouble() * 60;
  }

  public void setIntakeArmPosition(double targetPosition) {
    intakeArm.setControl(intakeArmPosition.withPosition(targetPosition));
  }

  public double getIntakePosition() {
    return intakeArm.getPosition().getValueAsDouble();
  }

  public void setIntakeRollerRPM(double targetRPM) {
    intakeRoller.setControl(intakeRollerRPM.withVelocity(targetRPM / 60));
  }

  public double getIntakeRollerRPM() {
    return intakeRoller.getVelocity().getValueAsDouble() * 60;
  }
}
