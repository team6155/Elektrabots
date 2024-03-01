// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DriveConstants.FRONT_LEFT_ANGULAR_OFFSET,
    "Front Left"
  );

  private final SwerveModule FRONT_RIGHT = new SwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.FRONT_RIGHT_ANGULAR_OFFSET,
    "Front Right"
  );

  private final SwerveModule REAR_LEFT = new SwerveModule(
    DriveConstants.REAR_LEFT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_LEFT_ANGULAR_OFFSET,
    "Rear Left"
  );

  private final SwerveModule REAR_RIGHT = new SwerveModule(
    DriveConstants.REAR_RIGHT_DRIVING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
    DriveConstants.REAR_RIGHT_ANGULAR_OFFSET,
    "Rear Right"
  );
  
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void stopModules() {
    FRONT_LEFT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    FRONT_RIGHT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);;
    REAR_LEFT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);;
    REAR_RIGHT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean optimize, double speedRatio) {
    SmartDashboard.putString("Desired speeds", desiredStates[0].toString());
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    SmartDashboard.putString("Desaturated speeds", desiredStates[0].toString());
    FRONT_LEFT.setDesiredState(desiredStates[0], optimize);
    FRONT_RIGHT.setDesiredState(desiredStates[1], optimize);
    REAR_LEFT.setDesiredState(desiredStates[2], optimize);
    REAR_RIGHT.setDesiredState(desiredStates[3], optimize);
  }

  public void stop() {
    FRONT_LEFT.stop();
    FRONT_RIGHT.stop();
    REAR_LEFT.stop();
    REAR_RIGHT.stop();
  }
  
  public void testMotorDriving() {
    FRONT_LEFT.testDrive();
    FRONT_RIGHT.testDrive();
    REAR_LEFT.testDrive();
    REAR_RIGHT.testDrive();
  }

  public void testMotorRotating() {
    FRONT_LEFT.testRotation();
    FRONT_RIGHT.testRotation();
    REAR_LEFT.testRotation();
    REAR_RIGHT.testRotation();
  }
}
