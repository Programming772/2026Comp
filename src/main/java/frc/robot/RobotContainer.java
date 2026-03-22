// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleoperatedControlCommand;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // final DoubleSolenoid climber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);

  final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  // final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // final VisionSubsystem visionSubsystem = new VisionSubsystem();

  final Joystick driverController = new Joystick(0);
  SendableChooser<Command> autoChooser;

  Trigger climbToggle = new JoystickButton(driverController, 1);
  Trigger slow = new JoystickButton(driverController, 6);
  Trigger hold45 = new JoystickButton(driverController, 2);
  Trigger resetHeading = new JoystickButton(driverController, 7);

  private RobotConfig config;

  public RobotContainer() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config", e.getStackTrace());
    }

    // instantiates the PathPlanner AutoBuilder 
    AutoBuilder.configure(
      swerveSubsystem::getPose, 
      swerveSubsystem::autoPose, 
      swerveSubsystem::getRobotSpeeds, 
      swerveSubsystem::setSpeeds, 
      new PPHolonomicDriveController(
        new PIDConstants(
          Constants.SwerveConstants.propulsionPIDkp, 
          Constants.SwerveConstants.propulsionPIDki, 
          Constants.SwerveConstants.propulsionPIDkd), 
        new PIDConstants(
          Constants.SwerveConstants.turningPIDkp, 
          Constants.SwerveConstants.turningPIDki, 
          Constants.SwerveConstants.turningPIDkd)
      ), 
      config, 
      () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red, 
      swerveSubsystem
    );

    // climber.set(Value.kForward);

    swerveSubsystem.setDefaultCommand(new TeleoperatedControlCommand(
      swerveSubsystem,
      () -> !driverController.getRawButton(7),
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> driverController.getRawAxis(4)
    )); 

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // climbToggle.onTrue(new InstantCommand(() -> climber.toggle()));
    slow.whileTrue(new InstantCommand(() -> swerveSubsystem.slowSpeed())).onFalse(new InstantCommand(() -> swerveSubsystem.regularSpeed()));
    hold45.onTrue(new InstantCommand(() -> swerveSubsystem.humpRot()));
    resetHeading.onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
