// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimWithVisionCommand;
import frc.robot.commands.TeleoperatedControlCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // final DoubleSolenoid climber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);

  final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  final VisionSubsystem visionSubsystem = new VisionSubsystem();

  final Joystick driverController = new Joystick(0);
  final Joystick testController = new Joystick(1);
  final Joystick manualControls = new Joystick(2);

  SendableChooser<Command> autoChooser;

  // driver controls
  Trigger climbToggle = new JoystickButton(driverController, 1);
  Trigger hold45 = new JoystickButton(driverController, 2);
  Trigger intakeToggle = new JoystickButton(driverController, 3);
  Trigger slow = new JoystickButton(driverController, 8);
  Trigger resetHeading = new JoystickButton(driverController, 7);
  Trigger feedingToggle = new JoystickButton(driverController, 4);
  Trigger midShooting = new JoystickButton(driverController, 5);
  Trigger toggleShooting = new JoystickButton(driverController, 6);

  // test controls
  Trigger hoodTest = new JoystickButton(testController, 1);
  Trigger turretTest = new JoystickButton(testController, 2);
  Trigger towerTest = new JoystickButton(testController, 3);
  Trigger flywheelTest = new JoystickButton(testController, 4);
  
  Trigger feederTest = new JoystickButton(testController, 7);
  Trigger intakeArmTest = new JoystickButton(testController, 5);
  Trigger intakeRollersTest = new JoystickButton(testController, 6);

  // manual controls
  Trigger slowTower = new JoystickButton(manualControls, 1);
  Trigger slowFlywheel = new JoystickButton(manualControls, 2);
  Trigger slowFeeder = new JoystickButton(manualControls, 3);
  Trigger slowIntakeRollers = new JoystickButton(manualControls, 4);
  Trigger turretCW = new JoystickButton(manualControls, 5);
  Trigger turretCCW = new JoystickButton(manualControls, 6);
  Trigger intakeUp = new JoystickButton(manualControls, 7);
  Trigger intakeDown = new JoystickButton(manualControls, 8);
  Trigger hoodFwd = new JoystickButton(manualControls, 9);
  Trigger hoodRvs = new JoystickButton(manualControls, 10);


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
      () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0),
      () -> driverController.getRawAxis(4)
    ));

    shooterSubsystem.setDefaultCommand(new AimWithVisionCommand(shooterSubsystem, visionSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // driver controls
    // climbToggle.onTrue(new InstantCommand(() -> climber.toggle()));
    
    slow.whileTrue(new InstantCommand(() -> swerveSubsystem.slowSpeed()))
    .onFalse(new InstantCommand(() -> swerveSubsystem.regularSpeed()));

    hold45.onTrue(new InstantCommand(() -> swerveSubsystem.humpRot()));
    resetHeading.onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    intakeToggle.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(0)))
    .toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(0)));
    
    feedingToggle.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(6000))
    .alongWith(new InstantCommand(() -> intakeSubsystem.setFeederRPM(6000)))
    .alongWith(new InstantCommand(() -> shooterSubsystem.setTowerRPM(6000))))
    .toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(0))
    .alongWith(new InstantCommand(() -> intakeSubsystem.setFeederRPM(0)))
    .alongWith(new InstantCommand(() -> shooterSubsystem.setTowerRPM(0))));

    midShooting.onTrue(new InstantCommand(() -> shooterSubsystem.setInMid(true)))
    .onFalse(new InstantCommand(() -> shooterSubsystem.setInMid(false)));

    toggleShooting.toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setCanShoot(false)))
    .toggleOnFalse(new InstantCommand(() -> shooterSubsystem.setCanShoot(true)));

    // test controls
    hoodTest.toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setHoodPosition(100))).toggleOnFalse(new InstantCommand(() -> shooterSubsystem.setHoodPosition(0)));
    turretTest.toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setTurretPosition(Rotation2d.fromDegrees(90)))).toggleOnFalse(new InstantCommand(() -> shooterSubsystem.setTurretPosition(Rotation2d.fromDegrees(0))));
    towerTest.toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setTowerRPM(6000))).toggleOnFalse(new InstantCommand(() -> shooterSubsystem.setTowerRPM(0)));
    flywheelTest.toggleOnTrue(new InstantCommand(() -> shooterSubsystem.setflywheelRPM(6000))).toggleOnFalse(new InstantCommand(() -> shooterSubsystem.setflywheelRPM(0)));
    
    feederTest.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setFeederRPM(6000))).toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setFeederRPM(0)));
    intakeArmTest.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(100))).toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setIntakeArmPosition(0)));
    intakeRollersTest.toggleOnTrue(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(6000))).toggleOnFalse(new InstantCommand(() -> intakeSubsystem.setIntakeRollerRPM(0)));

    // manual controls
    slowTower.onTrue(new InstantCommand(() -> shooterSubsystem.manualTower(10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualTower(0)));
    slowFlywheel.onTrue(new InstantCommand(() -> shooterSubsystem.manualFlywheel(10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualFlywheel(0)));
    slowFeeder.onTrue(new InstantCommand(() -> intakeSubsystem.manualFeeder(10))).onFalse(new InstantCommand(() -> intakeSubsystem.manualFeeder(0)));
    slowIntakeRollers.onTrue(new InstantCommand(() -> intakeSubsystem.manualIntakeRoller(10))).onFalse(new InstantCommand(() -> intakeSubsystem.manualIntakeRoller(0)));
    turretCW.onTrue(new InstantCommand(() -> shooterSubsystem.manualTurret(10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualTurret(0)));
    turretCCW.onTrue(new InstantCommand(() -> shooterSubsystem.manualTurret(-10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualTurret(0)));
    intakeUp.onTrue(new InstantCommand(() -> intakeSubsystem.manualIntakePosition(10))).onFalse(new InstantCommand(() -> intakeSubsystem.manualIntakePosition(0)));
    intakeDown.onTrue(new InstantCommand(() -> intakeSubsystem.manualIntakePosition(-10))).onFalse(new InstantCommand(() -> intakeSubsystem.manualIntakePosition(0)));
    hoodFwd.onTrue(new InstantCommand(() -> shooterSubsystem.manualHood(10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualHood(0)));
    hoodRvs.onTrue(new InstantCommand(() -> shooterSubsystem.manualHood(-10))).onFalse(new InstantCommand(() -> shooterSubsystem.manualHood(0)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
