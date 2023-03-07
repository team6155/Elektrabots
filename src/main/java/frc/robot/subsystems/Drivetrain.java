// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Subsystem of the robot controlling driving.
 * 
 * This robot uses four swerve modules as its wheels.
 */
public class Drivetrain extends SubsystemBase {
  public boolean idle;

  private final SwerveModule FRONT_LEFT = new SwerveModule(
    DriveConstants.FRONT_LEFT_DRIVING_MOTOR_PORT,
    DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
    DriveConstants.FRONT_LEFT_DRIVING_MOTOR_REVERSED,
    DriveConstants.FRONT_LEFT_TURNING_MOTOR_REVERSED,
    DriveConstants.FRONT_LEFT_TURNING_ENCODER_PORTS,
    "Front Left"
  );

  private final SwerveModule FRONT_RIGHT = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_REVERSED,
    DriveConstants.FRONT_RIGHT_TURNING_MOTOR_REVERSED,
    DriveConstants.FRONT_RIGHT_TURNING_ENCODER_PORTS,
    "Front Right"
  );

  private final SwerveModule REAR_LEFT = new SwerveModule(
    DriveConstants.REAR_LEFT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_DRIVING_MOTOR_REVERSED,
    DriveConstants.REAR_LEFT_TURNING_MOTOR_REVERSED,
    DriveConstants.REAR_LEFT_TURNING_ENCODER_PORTS,
    "Rear Left"
  );

  private final SwerveModule REAR_RIGHT = new SwerveModule(
    DriveConstants.REAR_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_DRIVING_MOTOR_REVERSED,
    DriveConstants.REAR_RIGHT_TURNING_MOTOR_REVERSED,
    DriveConstants.REAR_RIGHT_TURNING_ENCODER_PORTS,
    "Rear Right"
  );

  private final Gyro GYRO = new ADXRS450_Gyro();
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  public void stopModules() {
    FRONT_LEFT.stop();
    FRONT_RIGHT.stop();
    REAR_LEFT.stop();
    REAR_RIGHT.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean optimize) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.SPEED_RATIO);
    FRONT_LEFT.setDesiredState(desiredStates[0], optimize);
    FRONT_RIGHT.setDesiredState(desiredStates[1], optimize);
    REAR_LEFT.setDesiredState(desiredStates[2], optimize);
    REAR_RIGHT.setDesiredState(desiredStates[3], optimize);
  }

  public void zeroHeading() {
    GYRO.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(GYRO.getAngle(), 360);
  }

  public double getTurnRate() {
    return GYRO.getRate() * (DriveConstants.GYRO_REVERSED ? -1 : 1);
  }

  public void test() {
    FRONT_LEFT.test();
    FRONT_RIGHT.test();
    REAR_LEFT.test();
    REAR_RIGHT.test();
  }
}
