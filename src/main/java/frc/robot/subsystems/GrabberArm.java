// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberArm extends SubsystemBase {
  private static final double SPEED_LIMIT = .5;

  private MotorController leftMotor;
  private MotorController rightMotor;

  /** Creates a new GrabberArm. */
  public GrabberArm() {
    leftMotor = new WPI_VictorSPX(GrabberConstants.ARM_LEFT_MOTOR_PORT);
    rightMotor = new WPI_VictorSPX(GrabberConstants.ARM_RIGHT_MOTOR_PORT);
  }

  public void run(double speed) {
    leftMotor.set(speed * SPEED_LIMIT);
    rightMotor.set(speed * SPEED_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
