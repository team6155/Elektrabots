// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

  private final MotorController DRIVING_MOTOR;
  private final MotorController TURNING_MOTOR;

  private final Encoder TURNING_ENCODER;

  private final ProfiledPIDController TURNING_PID_CONTROLLER;
  private final SlewRateLimiter RATE_LIMITER;

  private String name;

  /** Creates a new SwerveModule. */
  public SwerveModule(int drivingMotorChannel, int turningMotorChannel, boolean drivingMotorReversed, 
      boolean turningMotorReversed, int[] turningEncoderChannels, String name) {

    DRIVING_MOTOR = new WPI_VictorSPX(drivingMotorChannel);
    TURNING_MOTOR = new WPI_VictorSPX(turningMotorChannel);
    DRIVING_MOTOR.setInverted(drivingMotorReversed);
    TURNING_MOTOR.setInverted(turningMotorReversed);

    TURNING_ENCODER = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);
    TURNING_ENCODER.setDistancePerPulse(SwerveModuleConstants.TURNING_ENCODER_DISTANCE_PER_PULSE);

    TURNING_PID_CONTROLLER = new ProfiledPIDController(SwerveModuleConstants.TURNING_CONTROLLER_P_VALUE, 0, 0,
        SwerveModuleConstants.ROTATION_CONSTRAINTS);
    TURNING_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

    RATE_LIMITER = new SlewRateLimiter(.8);

    this.name = name;
  }

  public void setDesiredState(SwerveModuleState state, boolean optimize) {
    if (optimize) {
      state = SwerveModuleState.optimize(state, new Rotation2d(TURNING_ENCODER.getDistance()));
    }
    double driveOutput = RATE_LIMITER.calculate(state.speedMetersPerSecond);
    double turnOutput = TURNING_PID_CONTROLLER.calculate(TURNING_ENCODER.getDistance(), state.angle.getRadians());

    DRIVING_MOTOR.set(driveOutput);
    TURNING_MOTOR.set(turnOutput);
  }

  public void stop() {
    DRIVING_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }

  public void resetEncoders() {
    TURNING_ENCODER.reset();
  }

  public boolean resetWheel() {
    if (Math.abs(TURNING_ENCODER.getDistance()) <= InputConstants.DEADBAND) {
      TURNING_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
      return true;
    }
    TURNING_PID_CONTROLLER.disableContinuousInput();
    TURNING_MOTOR.set(TURNING_PID_CONTROLLER.calculate(TURNING_ENCODER.getDistance(), 0));
    return false;
  }

  public void test() {
    DRIVING_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }
}
