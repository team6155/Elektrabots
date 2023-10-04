// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.InputConstants;

public class GrabberWrist extends SubsystemBase {
  private static final double SPEED_LIMIT = 1;
  private final SlewRateLimiter RATE_LIMITER;
  private MotorController motor;

  /** Creates a new GrabberWrist. */
  public GrabberWrist() {
    RATE_LIMITER = new SlewRateLimiter(InputConstants.ACCELERATION_RATE_LIMIT);
    motor = new WPI_VictorSPX(GrabberConstants.WRIST_MOTOR_PORT);
  }

  public void run(double speed) {
    motor.set(RATE_LIMITER.calculate(speed * SPEED_LIMIT));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
