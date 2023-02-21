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
  private final SwerveModule FRONT_LEFT = new SwerveModule(
    DriveConstants.FRONT_LEFT_DRIVING_MOTOR_PORT,
    DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
    DriveConstants.FRONT_LEFT_DRIVING_MOTOR_REVERSED,
    DriveConstants.FRONT_LEFT_TURNING_MOTOR_REVERSED,
    DriveConstants.FRONT_LEFT_DRIVING_ENCODER_PORTS,
    DriveConstants.FRONT_LEFT_TURNING_ENCODER_PORTS
  );

  private final SwerveModule FRONT_RIGHT = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_REVERSED,
    DriveConstants.FRONT_RIGHT_TURNING_MOTOR_REVERSED,
    DriveConstants.FRONT_RIGHT_DRIVING_ENCODER_PORTS,
    DriveConstants.FRONT_RIGHT_TURNING_ENCODER_PORTS
  );

  private final SwerveModule REAR_LEFT = new SwerveModule(
    DriveConstants.REAR_LEFT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_DRIVING_MOTOR_REVERSED,
    DriveConstants.REAR_LEFT_TURNING_MOTOR_REVERSED,
    DriveConstants.REAR_LEFT_DRIVING_ENCODER_PORTS,
    DriveConstants.REAR_LEFT_TURNING_ENCODER_PORTS
  );

  private final SwerveModule REAR_RIGHT = new SwerveModule(
    DriveConstants.REAR_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_DRIVING_MOTOR_REVERSED,
    DriveConstants.REAR_RIGHT_TURNING_MOTOR_REVERSED,
    DriveConstants.REAR_RIGHT_DRIVING_ENCODER_PORTS,
    DriveConstants.REAR_RIGHT_TURNING_ENCODER_PORTS
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

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    FRONT_LEFT.setDesiredState(desiredStates[0]);
    FRONT_RIGHT.setDesiredState(desiredStates[1]);
    REAR_LEFT.setDesiredState(desiredStates[2]);
    REAR_RIGHT.setDesiredState(desiredStates[3]);
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
}
