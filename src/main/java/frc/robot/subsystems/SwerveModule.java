// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

  private final CANSparkMax DRIVING_MOTOR;
  private final CANSparkMax TURNING_MOTOR;

  private final RelativeEncoder DRIVING_ENCODER;
  private final AbsoluteEncoder TURNING_ENCODER;

  private final SparkPIDController DRIVING_PID_CONTROLLER;
  private final SparkPIDController TURNING_PID_CONTROLLER;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

  private String name;

  /** Creates a new SwerveModule and configures the motors, encoders, and PID controllers. */
  public SwerveModule(int drivingMotorChannel, int turningMotorChannel, double chassisAngularOffset, String name) {

    // Set up motors
    DRIVING_MOTOR = new CANSparkMax(drivingMotorChannel, MotorType.kBrushless);
    DRIVING_MOTOR.setIdleMode(IdleMode.kBrake);
    DRIVING_MOTOR.setSmartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);

    TURNING_MOTOR = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    TURNING_MOTOR.setIdleMode(IdleMode.kBrake);
    TURNING_MOTOR.setSmartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save motor configuration in case of a brown out.
    DRIVING_MOTOR.burnFlash();
    TURNING_MOTOR.burnFlash();

  
    /**
     * Set up Encoders
     * We need do unit conversion on the encoders' position and velocity numbers.
     * The default units are rotations, but for the driving encoder, we need meters,
     * and for the turning encoder, we need radians.
     */
    DRIVING_ENCODER = DRIVING_MOTOR.getEncoder();
    DRIVING_ENCODER.setPositionConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
    DRIVING_ENCODER.setVelocityConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR / 60);

    TURNING_ENCODER = TURNING_MOTOR.getAbsoluteEncoder(Type.kDutyCycle);
    TURNING_ENCODER.setPositionConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    TURNING_ENCODER.setVelocityConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR / 60);
    TURNING_ENCODER.setInverted(SwerveModuleConstants.TURNING_ENCODER_INVERTED);


    /** Set up PID Controllers */
    DRIVING_PID_CONTROLLER = DRIVING_MOTOR.getPIDController();
    DRIVING_PID_CONTROLLER.setFeedbackDevice(DRIVING_ENCODER);
    DRIVING_PID_CONTROLLER.setP(SwerveModuleConstants.DRIVING_P_VALUE);
    DRIVING_PID_CONTROLLER.setI(SwerveModuleConstants.DRIVING_I_VALUE);
    DRIVING_PID_CONTROLLER.setD(SwerveModuleConstants.DRIVING_D_VALUE);
    DRIVING_PID_CONTROLLER.setFF(SwerveModuleConstants.DRIVING_FF_VALUE);
    DRIVING_PID_CONTROLLER.setOutputRange(-1, 1);

    TURNING_PID_CONTROLLER = TURNING_MOTOR.getPIDController();
    TURNING_PID_CONTROLLER.setFeedbackDevice(TURNING_ENCODER);
    TURNING_PID_CONTROLLER.setPositionPIDWrappingEnabled(true);
    TURNING_PID_CONTROLLER.setPositionPIDWrappingMinInput(0);
    TURNING_PID_CONTROLLER.setPositionPIDWrappingMaxInput(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    TURNING_PID_CONTROLLER.setP(SwerveModuleConstants.TURNING_P_VALUE);
    TURNING_PID_CONTROLLER.setI(SwerveModuleConstants.TURNING_I_VALUE);
    TURNING_PID_CONTROLLER.setD(SwerveModuleConstants.TURNING_D_VALUE);
    TURNING_PID_CONTROLLER.setFF(SwerveModuleConstants.TURNING_FF_VALUE);
    TURNING_PID_CONTROLLER.setOutputRange(-1, 1);
    

    this.name = name;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(DRIVING_ENCODER.getVelocity(),
        new Rotation2d(TURNING_ENCODER.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        DRIVING_ENCODER.getPosition(),
        new Rotation2d(TURNING_ENCODER.getPosition() - chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState state, boolean optimize) {
    state.angle = state.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    if (optimize) {
      state = SwerveModuleState.optimize(state, new Rotation2d(TURNING_ENCODER.getPosition()));
    }

    DRIVING_PID_CONTROLLER.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    TURNING_PID_CONTROLLER.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    desiredState = state;
  }

  public void stop() {
    DRIVING_MOTOR.set(0);
    TURNING_MOTOR.set(0);
  }

  public void testDrive() {
    DRIVING_MOTOR.set(.2);
  }

  public void testRotation() {
    TURNING_MOTOR.set(.2);
  }
}
