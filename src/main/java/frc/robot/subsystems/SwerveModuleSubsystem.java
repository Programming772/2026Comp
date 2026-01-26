// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleSubsystem extends SubsystemBase {
  // objects and variables instantiation
  public TalonFX m_propulsionMotor, m_turningMotor;
  public CANcoder m_encoder;
  public Angle m_encoderOffset;
  public boolean m_encoderReversed;

  public PIDController propulsionPID = new PIDController(SwerveConstants.propulsionPIDkp, SwerveConstants.propulsionPIDki, SwerveConstants.propulsionPIDkd);
  public ProfiledPIDController turningPID = new ProfiledPIDController(SwerveConstants.turningPIDkp, SwerveConstants.turningPIDki, SwerveConstants.turningPIDkd, new TrapezoidProfile.Constraints(SwerveConstants.maxAngularVelocity, SwerveConstants.maxAngularAcceleration));

  public SimpleMotorFeedforward propulsionFF = new SimpleMotorFeedforward(SwerveConstants.propulsionFFks, SwerveConstants.propulsionFFkv, SwerveConstants.propulsionFFka);
  public SimpleMotorFeedforward turningFF = new SimpleMotorFeedforward(SwerveConstants.turningFFks, SwerveConstants.turningFFkv, SwerveConstants.turningFFka);

  public SwerveModuleSubsystem(int propulsionMotorID, int turningMotorID, int encoderID, Angle encoderOffset,
      boolean encoderReversed, InvertedValue driveInverted, InvertedValue turningInverted) {
    m_propulsionMotor = new TalonFX(propulsionMotorID);
    m_turningMotor = new TalonFX(turningMotorID);

    // defines configs for the turning motors
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 100;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveInverted;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;

    // defines configs for the turning motors
    TalonFXConfiguration turningConfig = new TalonFXConfiguration();
    turningConfig.CurrentLimits.StatorCurrentLimit = 50;
    turningConfig.CurrentLimits.SupplyCurrentLimit = 70;
    turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turningConfig.MotorOutput.Inverted = turningInverted;
    turningConfig.Voltage.PeakForwardVoltage = 12.0;
    turningConfig.Voltage.PeakReverseVoltage = -12.0;
    turningConfig.Feedback.FeedbackRemoteSensorID = encoderID;
    turningConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    // applies the configs to the motors
    m_propulsionMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_propulsionMotor.getConfigurator().apply(driveConfig);
    m_turningMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_turningMotor.getConfigurator().apply(turningConfig);

    // makes a new absolute encoder, get the
    m_encoder = new CANcoder(encoderID);
    m_encoderOffset = encoderOffset;
    m_encoderReversed = encoderReversed;

    // makes a PID controller for turning module
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SwerveModule [" + m_encoder.getDeviceID() + "]", getCANCoder().getDegrees());
  }

  public double getPropulsionPosition() {
    // returns the position of the propulsion motor in meters
    return ((m_propulsionMotor.getPosition().getValueAsDouble()) / Constants.SwerveConstants.kDriveGearRatio)
        * (Constants.SwerveConstants.wheelRadiusMeters * 2 * Math.PI);
  }

  public double getPropulsionVelocity() {
    // returns the velocity of the propulsion motor in meters per second
    return ((m_propulsionMotor.getVelocity().getValueAsDouble()) / Constants.SwerveConstants.kDriveGearRatio)
        * (Constants.SwerveConstants.wheelRadiusMeters * 2 * Math.PI);
  }

  public void stopMotors() {
    // sets the motor speed to 0 to turn the off
    m_propulsionMotor.set(0);
    m_turningMotor.set(0);
  }

  public Rotation2d getCANCoder() {
    // returns the position of the absolute encoder in radians
    return Rotation2d.fromRotations((m_encoder.getAbsolutePosition().getValueAsDouble()) * (m_encoderReversed ? -1 : 1) - m_encoderOffset.in(Units.Rotations));
  }

  public double getCanCoderVelocity() {
    return (m_encoder.getVelocity().getValueAsDouble()) * 2 * Math.PI;
  }

  public void resetEncoders() {
    // sets encoders to 0
    m_propulsionMotor.setPosition(0);
    m_turningMotor.setPosition(getCANCoder().getRotations());
  }

  public SwerveModuleState getState() {
    // returns the current state of the module (speed and rotation)
    return new SwerveModuleState(getPropulsionVelocity(), getCANCoder());
  }

  public SwerveModulePosition getPosition() {
    // returns the current position of the module (distance traveled and rotation)
    return new SwerveModulePosition(getPropulsionPosition(), getCANCoder());
  }

  private SwerveModuleState optimizeState(SwerveModuleState desired, Rotation2d currentAngle) {
    double delta = MathUtil.angleModulus(desired.angle.minus(currentAngle).getRadians());

    // If rotating more than 90°, flip wheel direction
    if (Math.abs(delta) > Math.PI / 2.0) {
      return new SwerveModuleState(-desired.speedMetersPerSecond, desired.angle.plus(Rotation2d.fromRadians(Math.PI)));
    }

    return desired;
  }

  public void setState(SwerveModuleState state) {
    // optimizes the rotation to the prefered angle
    state = optimizeState(state, getCANCoder());
    state.speedMetersPerSecond *= Math.cos(state.angle.minus(getCANCoder()).getRadians());

    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      m_propulsionMotor.setVoltage(0);
      return;
    }

    propulsionPID.setSetpoint(state.speedMetersPerSecond);
    turningPID.setGoal(state.angle.getRadians());

    m_propulsionMotor.setVoltage(propulsionPID.calculate(getPropulsionVelocity()) + propulsionFF.calculate(state.speedMetersPerSecond));
    m_turningMotor.setVoltage(turningPID.calculate(getCANCoder().getRadians()));
  }
}
