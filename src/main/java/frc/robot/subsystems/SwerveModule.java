// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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

  private final CANSparkMax DRIVING_MOTOR;
  private final CANSparkMax TURNING_MOTOR;

  private final RelativeEncoder DRIVING_ENCODER;
  private final AbsoluteEncoder TURNING_ENCODER;

  private final ProfiledPIDController TURNING_PID_CONTROLLER;
  private final SlewRateLimiter RATE_LIMITER;

  private String name;

  /** Creates a new SwerveModule. */
  public SwerveModule(int drivingMotorChannel, int turningMotorChannel, boolean drivingMotorReversed, 
      boolean turningMotorReversed, int[] drivingEncoderChannels, int[] turningEncoderChannels, String name) {

    DRIVING_MOTOR = new CANSparkMax(drivingMotorChannel, MotorType.kBrushless);
    TURNING_MOTOR = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    DRIVING_MOTOR.setInverted(drivingMotorReversed);
    TURNING_MOTOR.setInverted(turningMotorReversed);

    DRIVING_ENCODER = DRIVING_MOTOR.getEncoder();
    TURNING_ENCODER = TURNING_MOTOR.getAbsoluteEncoder(Type.kDutyCycle);

    TURNING_PID_CONTROLLER = new ProfiledPIDController(SwerveModuleConstants.TURNING_CONTROLLER_P_VALUE, 0, 0,
        SwerveModuleConstants.ROTATION_CONSTRAINTS);
    TURNING_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

    RATE_LIMITER = new SlewRateLimiter(.8);

    this.name = name;
  }

  public void setDesiredState(SwerveModuleState state, boolean optimize) {
    if (optimize) {
      state = SwerveModuleState.optimize(state, new Rotation2d(TURNING_ENCODER.getPosition()));
    }
    double driveOutput = RATE_LIMITER.calculate(state.speedMetersPerSecond);
    double turnOutput = TURNING_PID_CONTROLLER.calculate(TURNING_ENCODER.getPosition(), state.angle.getRadians());

    DRIVING_MOTOR.set(driveOutput);
    TURNING_MOTOR.set(turnOutput);
  }

  public void stop() {
    DRIVING_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }
}
