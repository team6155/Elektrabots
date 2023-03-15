// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.InputConstants;

public class GrabberArm extends SubsystemBase {
  private MotorController leftMotor;
  private MotorController rightMotor;
  private Encoder encoder;
  private ProfiledPIDController pidController;

  /** Creates a new GrabberArm. */
  public GrabberArm() {
    leftMotor = new WPI_VictorSPX(GrabberConstants.ARM_LEFT_MOTOR_PORT);
    rightMotor = new WPI_VictorSPX(GrabberConstants.ARM_RIGHT_MOTOR_PORT);
    leftMotor.setInverted(false);

    encoder = new Encoder(GrabberConstants.ARM_ENCODER_PORTS[0], GrabberConstants.ARM_ENCODER_PORTS[1]);
    encoder.setDistancePerPulse(GrabberConstants.ENCODER_DISTANCE_PER_PULSE);
    encoder.setReverseDirection(false);

    pidController = new ProfiledPIDController(1, 0, 0, GrabberConstants.ROTATION_CONSTRAINTS);
  }

  public void teleopRun(double speed) {
    if (Math.abs(speed) < InputConstants.DEADBAND) {
      speed = 0;
    }
    speed *= .6;
    double measurement = encoder.getDistance();
    double goal = MathUtil.clamp(measurement + speed, -.2, Math.PI);
    //run(goal);
    leftMotor.set(speed);
  
  }

  // TODO: Set PID loop back up.
  public void run(double goal) {
    double measurement = encoder.getDistance();
    double speed = MathUtil.clamp(pidController.calculate(measurement, goal), -.6, .6);
    leftMotor.set(speed);
  }

  public void resetEncoder() {
    encoder.reset();
  }
}
