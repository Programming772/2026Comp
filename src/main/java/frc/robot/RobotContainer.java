// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleoperatedControlCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
  final DoubleSolenoid climber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);

  final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  final Joystick driverController = new Joystick(0);

  Trigger climbToggle = new JoystickButton(driverController, 1);

  Trigger slow = new JoystickButton(driverController, 6);

  public RobotContainer() {
    climber.set(Value.kForward);

    swerveSubsystem.setDefaultCommand(new TeleoperatedControlCommand(
      swerveSubsystem,
      () -> !driverController.getRawButton(7),
      () -> driverController.getRawAxis(1),
      () -> driverController.getRawAxis(0),
      () -> driverController.getRawAxis(4)
    )); 

    configureBindings();
  }

  private void configureBindings() {
    climbToggle.onTrue(new InstantCommand(() -> climber.toggle()));
    slow.whileTrue(new InstantCommand(() -> swerveSubsystem.slowSpeed())).onFalse(new InstantCommand(() -> swerveSubsystem.regularSpeed()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
