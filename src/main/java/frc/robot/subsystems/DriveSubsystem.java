// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.WarriorGyro;

public class DriveSubsystem extends SubsystemBase {

  public static class DriveConstants {
    public static final int FRONT_LEFT_MODULE_DRIVE_CAN_ID = 12;
    public static final int FRONT_LEFT_MODULE_STEER_CAN_ID = 13;
    public static final int FRONT_LEFT_MODULE_ENCODER_CAN_ID = 13;
    /** Kitbot */
    public static final double FRONT_LEFT_MODULE_ROTATION_OFFSET = 0.2103; //0.2103

    public static final int FRONT_RIGHT_MODULE_DRIVE_CAN_ID = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_CAN_ID = 11;
    public static final int FRONT_RIGHT_MODULE_ENCODER_CAN_ID = 11;
    /** Kitbot */

    public static final double FRONT_RIGHT_MODULE_ROTATION_OFFSET =  0.0955; //.0955

    public static final int BACK_LEFT_MODULE_DRIVE_CAN_ID = 16;
    public static final int BACK_LEFT_MODULE_STEER_CAN_ID = 17;
    public static final int BACK_LEFT_MODULE_ENCODER_CAN_ID = 17;
    /** Kitbot */
    public static final double BACK_LEFT_MODULE_ROTATION_OFFSET = -0.2011; //-106.435

    public static final int BACK_RIGHT_MODULE_DRIVE_CAN_ID = 14;
    public static final int BACK_RIGHT_MODULE_STEER_CAN_ID = 15;
    public static final int BACK_RIGHT_MODULE_ENCODER_CAN_ID = 15;
    /** Kitbot */
    public static final double BACK_RIGHT_MODULE_ROTATION_OFFSET = 0.1802; //115.049 //

    /**
     * Drivetrain trackwidth from side to side
     * 
     * Measured from the center of the wheels on each side
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(18.75);

    /**
     * Drivetrain wheelbase from front to back
     * 
     * Measured from the center of the wheels on each side
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.75);

    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //public static final int COUNTS_PER_ROTATION = 2048;

    //public static final double STEER_MOTOR_ENCODER_COUNTS_PER_DEGREE = (STEER_GEAR_RATIO * COUNTS_PER_ROTATION)
    //    / 360;

    //public static final double DISTANCE_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE
    //    / GEAR_RATIO_MOTOR_TO_WHEEL / COUNTS_PER_ROTATION;

    // Formula for calculating theoretical max velocity:
    // Motor free speed RPM / 60 * Drive reduction * Wheel diameter meters * pi
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Constant for SDS MK4i Modules
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5; //(6380.0 / 60.0) * DRIVE_REDUCTION * DriveConstants.WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_VOLTAGE = 9.5; //TODO update with correct voltage
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 12.2;//MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2);
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; //TODO test value instead of theoretical value
    public static final double DRIVE_NERF_JOYSTICK_MULTIPLIER = 0.75;

    public static final double S_VOLTS = 0.01690; // 2/23/23 SysID said 0.20283, / 12 for our values
    public static final double V_VOLT_SECONDS_PER_METER = 0.1791; // 2/23/23 SysID said 2.1493, / 12 for our values
    public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.05486; // 2/23/23 SysID said 0.65828, / 12 for our values

    public static final double DRIVE_SLEW_RATE = 3; //2.5

    public static final double ANGLE_CONTROLLER_KP = 0.09;
  }
  
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  public final SwerveDriveKinematics swerveKinematics;

  private final SwerveDriveOdometry odometry;

  private final PIDController angleController;
  // private final PIDController driftCorrector;

  // Creates the slew rates to slowly accelerate controller inputs
  private SlewRateLimiter magnitudeSlewRate;
  private SlewRateLimiter xDriveSlew;
  private SlewRateLimiter yDriveSlew;

  private double modAngle;
  private double modVelocity;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontLeft = new SwerveModule("Front Left", DriveConstants.FRONT_LEFT_MODULE_DRIVE_CAN_ID,
        DriveConstants.FRONT_LEFT_MODULE_STEER_CAN_ID, DriveConstants.FRONT_LEFT_MODULE_ENCODER_CAN_ID,
        DriveConstants.FRONT_LEFT_MODULE_ROTATION_OFFSET);
    frontRight = new SwerveModule("Front Right", DriveConstants.FRONT_RIGHT_MODULE_DRIVE_CAN_ID,
        DriveConstants.FRONT_RIGHT_MODULE_STEER_CAN_ID, DriveConstants.FRONT_RIGHT_MODULE_ENCODER_CAN_ID,
        DriveConstants.FRONT_RIGHT_MODULE_ROTATION_OFFSET);
    backLeft = new SwerveModule("Back Left", DriveConstants.BACK_LEFT_MODULE_DRIVE_CAN_ID,
        DriveConstants.BACK_LEFT_MODULE_STEER_CAN_ID, DriveConstants.BACK_LEFT_MODULE_ENCODER_CAN_ID,
        DriveConstants.BACK_LEFT_MODULE_ROTATION_OFFSET);
    backRight = new SwerveModule("Back Right", DriveConstants.BACK_RIGHT_MODULE_DRIVE_CAN_ID,
        DriveConstants.BACK_RIGHT_MODULE_STEER_CAN_ID, DriveConstants.BACK_RIGHT_MODULE_ENCODER_CAN_ID,
        DriveConstants.BACK_RIGHT_MODULE_ROTATION_OFFSET);

    swerveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0,
            -DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0));

    odometry = new SwerveDriveOdometry(swerveKinematics, WarriorGyro.getYawAngle(), new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition() });

    // PID controller for the rotation of the robot
    angleController = new PIDController(DriveConstants.ANGLE_CONTROLLER_KP, 0, 0);
    angleController.enableContinuousInput(-180, 180);
    angleController.setTolerance(2);

    // driftCorrector = new PIDController(.001, 0, 0); // TODO implement Feed
    // Forward for functionality
    // driftCorrector.enableContinuousInput(0, 360);

    magnitudeSlewRate = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

    Shuffleboard.getTab("Drive Subsystem").add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(WarriorGyro.getYawAngle(), new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
    });
  }

  // ODOMETRY METHODS \\

  /**
   * Returns estimated current robot pose in meters
   * 
   * @return current robot pose2d in meters
   */
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns estimated current robot heading as a Rotation2d
   * 
   * @return current estimated robot heading as a Rotation2d (radians)
   */
  public Rotation2d getPoseHeading() {
    return odometry.getPoseMeters().getRotation();
  }

  /**
   * Resets odometry to the given pose value
   * 
   * @param pose to use for reset
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(WarriorGyro.getYawAngle(), new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition() }, pose);
  }

  /**
   * Sets the swerve module states
   * 
   * @param desiredStates the desired swerve module states
   */
  public void setTeleopModuleStates(SwerveModuleState[] desiredStates) {
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    // Sets the swerve modules to their desired states using optimization method

    modVelocity = desiredStates[0].speedMetersPerSecond;
    modAngle = desiredStates[0].angle.getDegrees();
    frontLeft.setTeleopDesiredState(desiredStates[0]);
    frontRight.setTeleopDesiredState(desiredStates[1]);
    backLeft.setTeleopDesiredState(desiredStates[2]);
    backRight.setTeleopDesiredState(desiredStates[3]);
  }

  /**
   * Sets the swerve module states
   * Uses closed loop control for auto
   * 
   * @param desiredStates the desired swerve module states
   */
  public void setAutoModuleStates(SwerveModuleState[] desiredStates) {
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    // Sets the swerve modules to their desired states using optimization method
    frontLeft.setAutoDesiredState(desiredStates[0]);
    frontRight.setAutoDesiredState(desiredStates[1]);
    backLeft.setAutoDesiredState(desiredStates[2]);
    backRight.setAutoDesiredState(desiredStates[3]);
  }

  /**
   * Sets the desired ChassisSpeeds
   * Used for auto
   * 
   * @param chassisSpeeds desired ChassisSpeeds
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, WarriorGyro.getYawAngle());

    // Sets field relative speeds
    var swerveModuleStates = swerveKinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, WarriorGyro.getYawAngle()));
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

    // Sets the swerve modules to their desired states using optimization method
    frontLeft.setTeleopDesiredState(swerveModuleStates[0]);
    frontRight.setTeleopDesiredState(swerveModuleStates[1]);
    backLeft.setTeleopDesiredState(swerveModuleStates[2]);
    backRight.setTeleopDesiredState(swerveModuleStates[3]);
  }

  // TODO not currently used
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  /**
   * Sets the steer motors to the absolute encoder positions
   */
  public void setSteerMotorsToAbsolute() {
    frontRight.setSteerMotorToAbsolute();
    frontLeft.setSteerMotorToAbsolute();
    backLeft.setSteerMotorToAbsolute();
    backRight.setSteerMotorToAbsolute();
  }

  /**
   * Same as drive method, but without the trigger control inputs.
   * This allows it to be used for driving to targets based on vision.
   * 
   * @param xSpeedInput   percent input from -1 to 1 (converts to meters per sec)
   * @param ySpeedInput   percent input from -1 to 1 (converts to meters per sec)
   * @param rotationInput percent input from -1 to 1 (converts to radians per sec)
   */
  public void autoDrive(double xSpeedInput, double ySpeedInput, double rotationInput) {
    // double xSpeed = xSpeedInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;
    // double ySpeed = ySpeedInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND;

    // double rotation = rotationInput *
    // DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    double xSpeed = xSpeedInput * (DriveConstants.MAX_VELOCITY_METERS_PER_SECOND / 2) + Math.signum(xSpeedInput) * 0.18;
    double ySpeed = ySpeedInput * (DriveConstants.MAX_VELOCITY_METERS_PER_SECOND / 2) + Math.signum(ySpeedInput) * 0.18;

    double rotation = rotationInput * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND + Math.signum(rotationInput) * 0.18;

    // Sets field relative speeds
    var swerveModuleStates = swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, WarriorGyro.getYawAngle()));
    // Ensures all wheels obey max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

    // Sets the swerve modules to their desired states using optimization method
    frontLeft.setTeleopDesiredState(swerveModuleStates[0]);
    frontRight.setTeleopDesiredState(swerveModuleStates[1]);
    backLeft.setTeleopDesiredState(swerveModuleStates[2]);
    backRight.setTeleopDesiredState(swerveModuleStates[3]);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Mod Velocity", () -> modVelocity, null);
    builder.addDoubleProperty("Mod Angle", () -> modAngle, null);
  }

}
