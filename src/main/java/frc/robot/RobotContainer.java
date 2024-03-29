// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems \\
  DriveSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  // Commands \\
  
  // Controllers \\
  CommandXboxController driverController;

  // Variables \\
  private static final double driveNerf = 0.85;
  private static final double steerNerf = 0.8;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new WarriorGyro();
    
    driveSubsystem = new DriveSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    driverController = new CommandXboxController(0);

    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem,
      () -> driverController.getLeftY() * driveNerf,
      () -> driverController.getLeftX() * driveNerf,
      () -> driverController.getRightX() * steerNerf,
      () -> driverController.a().getAsBoolean(),
      () -> driverController.b().getAsBoolean(),
      () -> driverController.x().getAsBoolean(),
      () -> driverController.y().getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.leftBumper().whileTrue(new InstantCommand(() -> intakeSubsystem.setIntakeMotorInput(IntakeConstants.INTAKE_IN_SPEED)));
    driverController.leftBumper().whileFalse(new InstantCommand(() -> intakeSubsystem.stopIntakeMotor()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
