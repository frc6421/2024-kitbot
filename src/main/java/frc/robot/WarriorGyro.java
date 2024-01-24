// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class WarriorGyro {

  public static class GyroConstants {
    public static final int GYRO_CAN_ID = 30;

    public static final double GYRO_PITCH_OFFSET = -0.056;
    public static final double GYRO_ROLL_OFFSET = 0.102;
    public static final double GYRO_YAW_OFFSET = -0.435;
  }

  private static Pigeon2 pigeon;
  
  /**
   * Constructs a pigeon gyro
   */
  public WarriorGyro() {
    pigeon = new Pigeon2(GyroConstants.GYRO_CAN_ID);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();
  }

  /**
   * Get the current Rotation2D rotation of the robot
   * 
   * @return Current robot rotation in Rotation2D
   */
  public static Rotation2d getStaticRotation() {
    return(pigeon.getRotation2d());
  }

  /**
   * Zeros the gyro
   */
  public static void zeroGyro() {
    pigeon.setYaw(GyroConstants.GYRO_YAW_OFFSET);
  }

  /**
   * Get rate of gyro rotation in degrees per second
   * 
   * @return turn rate in degrees per second
   */
  public static double getGyroRate() {
    return pigeon.getRate();
  }

  /**
   * Gets the pitch angle of the gyro
   * 
   * @return pitch angle in Rotation2d
   */
  public static Rotation2d getPitchAngle() {
    return Rotation2d.fromDegrees(pigeon.getPitch().getValue());
  }

  /**
   * Gets the yaw angle of the gyro
   * 
   * @return yaw angle in Rotation2d (radians)
   */
  public static Rotation2d getYawAngle() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
  }

  public static double getPitchAngleDouble() {
    return pigeon.getPitch().getValue();
  }

  /**
   * Gets the roll angle of the gyro
   * 
   * @return roll angle in Rotation2d
   */
  public static Rotation2d getRollAngle() {
    return Rotation2d.fromDegrees(pigeon.getRoll().getValue());
  }

}
