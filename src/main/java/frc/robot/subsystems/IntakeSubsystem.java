// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public static class IntakeConstants{
    public static int INTAKE_TOP_MOTOR_CAN_ID = 30;
    public static int INTAKE_BOTTOM_MOTOR_CAN_ID = 31;

    public static double INTAKE_IN_SPEED = 0.8;
    public static double INTAKE_OUT_SPEED = -0.8;
  }

  private VictorSPX intakeTop;
  private VictorSPX intakeBottom;
  private Double percent = 0.0;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeTop = new VictorSPX(IntakeConstants.INTAKE_TOP_MOTOR_CAN_ID);
    intakeBottom = new VictorSPX(IntakeConstants.INTAKE_BOTTOM_MOTOR_CAN_ID);

    Shuffleboard.getTab("Intake Subsystem").add(this);
  }

  public void setIntakeMotorInput(double value) {
    intakeTop.set(ControlMode.PercentOutput, -value);
    intakeBottom.set(ControlMode.PercentOutput, -value);
  }

  public void stopIntakeMotor() {
    intakeTop.set(ControlMode.PercentOutput, 0);
    intakeBottom.set(ControlMode.PercentOutput, 0);
  }

  public void initsendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    //add values to sendable
    builder.addDoubleProperty("Current % Output", () -> percent, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
