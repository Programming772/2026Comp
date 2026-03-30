// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = new CANBus("CANivore");

  TalonFX feeder = new TalonFX(IntakeConstants.feederID);
  TalonFX intakeArm = new TalonFX(IntakeConstants.intakeArmID);
  TalonFX intakeRoller = new TalonFX(IntakeConstants.intakeRollerID, kCANBus);

  VelocityVoltage feederRPM = new VelocityVoltage(0);
  PositionVoltage intakeArmPosition = new PositionVoltage(0);
  VelocityVoltage intakeRollerRPM = new VelocityVoltage(0);

  double intakeSetpoint = 0;
  
  public IntakeSubsystem() {
    intakeArm.setPosition(0);
    intakeArm.optimizeBusUtilization();
    BaseStatusSignal.setUpdateFrequencyForAll(
      50,
      intakeArm.getPosition()
    );
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = 150;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feederConfig.Voltage.PeakForwardVoltage = 12.0;
    feederConfig.Voltage.PeakReverseVoltage = -12.0;

    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();
    intakeArmConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeArmConfig.Slot0.kG = 1.85;
    intakeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    intakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeArmConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeArmConfig.Voltage.PeakForwardVoltage = 12.0;
    intakeArmConfig.Voltage.PeakReverseVoltage = -12.0;
    intakeArmConfig.MotionMagic.MotionMagicAcceleration = 0;
    intakeArmConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    
    TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();
    intakeRollerConfig.CurrentLimits.StatorCurrentLimit = 100;
    intakeRollerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeRollerConfig.Voltage.PeakForwardVoltage = 12.0;
    intakeRollerConfig.Voltage.PeakReverseVoltage = -12.0;
    
    feeder.getConfigurator().apply(new TalonFXConfiguration());
    feeder.getConfigurator().apply(feederConfig);
    intakeArm.getConfigurator().apply(new TalonFXConfiguration());
    intakeArm.getConfigurator().apply(intakeArmConfig);
    intakeRoller.getConfigurator().apply(new TalonFXConfiguration());
    intakeRoller.getConfigurator().apply(intakeRollerConfig);
  }

  @Override
  public void periodic() {
    if (MathUtil.isNear(IntakeConstants.intakePos, getIntakePosition(), 0.1) && (intakeArmPosition.getPositionMeasure().magnitude() == IntakeConstants.intakePos)) {
      intakeArm.stopMotor();
    }

    SmartDashboard.putNumber("Feeder RPM", getFeederRPM());
    SmartDashboard.putNumber("Intake Pos", getIntakePosition());
    SmartDashboard.putNumber("Intake RPM", getIntakeRollerRPM());
  }

  public void setZero() {
    intakeArm.setPosition(0);
  }

  public void setFeederRPM(double targetRPM) {
    feeder.setControl(feederRPM.withVelocity(targetRPM / 60));
  }

  public double getFeederRPM() {
    return feeder.getVelocity().getValueAsDouble() * 60;
  }

  public void manualFeeder(double speed) {
    feeder.set(speed);
  }

  // public void setIntakeArmPosition(double targetPosition) {
  //   intakeArm.setControl(new PositionVoltage(targetPosition).withSlot(0));
  // }

  public double getIntakePosition() {
    return intakeArm.getPosition().getValueAsDouble();
  }

  public void manualIntakePosition(double speed) {
    intakeArm.set(speed);
  }

  public void setIntakeRollerRPM(double targetRPM) {
    intakeRoller.setControl(intakeRollerRPM.withVelocity(targetRPM / 60));
  }

  public double getIntakeRollerRPM() {
    return intakeRoller.getVelocity().getValueAsDouble() * 60;
  }

  public void manualIntakeRoller(double speed) {
    intakeRoller.set(speed);
  }

  public void moveIntake(double setPoint) {
    this.intakeSetpoint = setPoint;
    double currentPos = getIntakePosition();

    if (MathUtil.isNear(intakeSetpoint, currentPos, 0.05)) {
      manualIntakePosition(0);
    } else {
      manualIntakePosition(-0.2 * ((currentPos - intakeSetpoint) / Math.abs(currentPos - intakeSetpoint)));
    }
  }
}
