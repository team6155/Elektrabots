// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

  private final MotorController DRIVING_MOTOR;
  private final MotorController TURNING_MOTOR;

  private final Encoder DRIVING_ENCODER;
  private final Encoder TURNING_ENCODER;

  private final PIDController DRIVING_PID_CONTROLLER;
  private final ProfiledPIDController TURNING_PID_CONTROLLER;

  /** Creates a new SwerveModule. */
  public SwerveModule(int drivingMotorChannel, int turningMotorChannel, boolean drivingMotorReversed, 
      boolean turningMotorReversed, int[] drivingEncoderChannels, int[] turningEncoderChannels) {
        
    DRIVING_MOTOR = new Spark(drivingMotorChannel);
    TURNING_MOTOR = new Spark(turningMotorChannel);
    DRIVING_MOTOR.setInverted(drivingMotorReversed);
    TURNING_MOTOR.setInverted(turningMotorReversed);

    DRIVING_ENCODER = new Encoder(drivingEncoderChannels[0], drivingEncoderChannels[1]);
    TURNING_ENCODER = new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);
    DRIVING_ENCODER.setDistancePerPulse(SwerveModuleConstants.DRIVING_ENCODER_DISTANCE_PER_PULSE);
    TURNING_ENCODER.setDistancePerPulse(SwerveModuleConstants.TURNING_ENCODER_DISTANCE_PER_PULSE);

    DRIVING_PID_CONTROLLER = new PIDController(SwerveModuleConstants.DRIVING_CONTROLLER_P_VALUE, 0, 0);
    TURNING_PID_CONTROLLER = new ProfiledPIDController(SwerveModuleConstants.TURNING_CONTROLLER_P_VALUE, 0, 0,
        SwerveModuleConstants.ROTATION_CONSTRAINTS);
    TURNING_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(DRIVING_ENCODER.getRate(), new Rotation2d(TURNING_ENCODER.getDistance()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(DRIVING_ENCODER.getDistance(), new Rotation2d(TURNING_ENCODER.getDistance()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, new Rotation2d(TURNING_ENCODER.getDistance()));
    double driveOutput = DRIVING_PID_CONTROLLER.calculate(DRIVING_ENCODER.getRate(), state.speedMetersPerSecond);
    double turnOutput = TURNING_PID_CONTROLLER.calculate(TURNING_ENCODER.getDistance(), state.angle.getRadians());

    DRIVING_MOTOR.set(driveOutput);
    TURNING_MOTOR.set(turnOutput);
  }

  public void stop() {
    DRIVING_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }

  public void resetEncoders() {
    DRIVING_ENCODER.reset();
    TURNING_ENCODER.reset();
  }
}
