// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.WarriorGyro;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class DriveCommand extends Command {
  private DriveSubsystem driveSubsystem;

  private DoubleSupplier leftYSupplier;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier rightXSupplier;

  private BooleanSupplier angleSupplierA;
  private BooleanSupplier angleSupplierB;
  private BooleanSupplier angleSupplierX;
  private BooleanSupplier angleSupplierY;

  private double xSpeedInput;
  private double ySpeedInput;
  private double rotationInput;

  private double percentDeadband = 0.1;

  private double currentAngle;
  private double targetAngle;

  private double maxSpeed = DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;
  private double maxAngularRate = 2 * Math.PI;

  private PIDController angleController;

  //private ChassisSpeeds chassisSpeedsPrintOut;

  private final SwerveRequest.FieldCentric driveRequest; 

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX,
      BooleanSupplier angleA, BooleanSupplier angleB, BooleanSupplier angleX, BooleanSupplier angleY) {

    driveSubsystem = drive;
    leftYSupplier = leftY;
    leftXSupplier = leftX;
    rightXSupplier = rightX;

    angleSupplierA = angleA;
    angleSupplierB = angleB;
    angleSupplierX = angleX;
    angleSupplierY = angleY;

    driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * percentDeadband).withRotationalDeadband(maxAngularRate * percentDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    //chassisSpeedsPrintOut = new ChassisSpeeds();

    // PID controller for the rotation of the robot
    // angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
    // angleController.enableContinuousInput(-180, 180);
    // angleController.setTolerance(2);

    Shuffleboard.getTab("SwerveModules").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // xSpeedInput = MathUtil.applyDeadband(leftYSupplier.getAsDouble(), percentDeadband);
    // ySpeedInput = MathUtil.applyDeadband(leftXSupplier.getAsDouble(), percentDeadband);
    // rotationInput = MathUtil.applyDeadband(rightXSupplier.getAsDouble(), percentDeadband);

    xSpeedInput = leftYSupplier.getAsDouble();
    ySpeedInput = leftXSupplier.getAsDouble();
    rotationInput = rightXSupplier.getAsDouble();

    double xSpeed;
    double ySpeed;
    double rotation;

    xSpeed = xSpeedInput;
    ySpeed = ySpeedInput;

    // xSpeed = -1 * (xSpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    // ySpeed = -1 * (ySpeedInput) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    currentAngle = WarriorGyro.getYawAngle().getDegrees();

    if (angleSupplierA.getAsBoolean()) {
      targetAngle = 180;

    } else if (angleSupplierB.getAsBoolean()) {
      targetAngle = -90;

    } else if (angleSupplierX.getAsBoolean()) {
      targetAngle = 90;

    } else if (angleSupplierY.getAsBoolean()) {
      targetAngle = 0;

    } else {
      targetAngle = currentAngle;

    }

    if (targetAngle != currentAngle) {
      rotation = angleController.calculate(WarriorGyro.getYawAngle().getDegrees(), targetAngle);
    } else {
    //   rotation = -1 * Math.signum(rotationInput) * (rotationInput * rotationInput)
    //       * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      rotation = rotationInput;
    }

    rotation = MathUtil.clamp(rotation, -2 * Math.PI, 2 * Math.PI);

    driveSubsystem.setControl(
        driveRequest.withVelocityX(-xSpeed * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC)
        .withVelocityY(-ySpeed * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC)
        .withRotationalRate(-rotation * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC));

    // Sets chassis speeds
    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation,
    //     WarriorGyro.getYawAngle());
    
    // chassisSpeedsPrintOut = chassisSpeeds;

    // // Sets field relative speeds to the swerve module states
    // var swerveModuleStates = driveSubsystem.

    // // Sets the swerve modules to their desired states using optimization method
    // driveSubsystem.setTeleopModuleStates(swerveModuleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // builder.addDoubleProperty("Gyro Degrees", () -> WarriorGyro.getYawAngle().getDegrees(), null);
    // builder.addDoubleProperty("Gyro Rotations", () -> WarriorGyro.getYawAngle().getRotations(), null);
    // builder.addDoubleProperty("Gyro Cos", () -> WarriorGyro.getYawAngle().getCos(), null);
    // builder.addDoubleProperty("Gyro Sin", () -> WarriorGyro.getYawAngle().getSin(), null);
    builder.addDoubleProperty("Target Angle", () -> targetAngle, null);
  }

}