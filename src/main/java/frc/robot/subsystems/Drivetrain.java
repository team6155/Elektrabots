// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

/**
 * Subsystem of the robot controlling driving.
 * 
 * This robot uses four swerve modules as its wheels.
 */
public class Drivetrain extends SubsystemBase {
  public boolean idle;

  // Slew rate filter variables for controlling lateral acceleration
  private double curMoveDirection = 0.0;
  private double curMoveMagnitude = 0.0;

  private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final ADXRS450_Gyro GYRO;

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
  public Drivetrain(ADXRS450_Gyro gyro) {
    GYRO = gyro;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param turningSpeed           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean fieldRelative, boolean rateLimit) {
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double newMoveDirection = Math.atan2(ySpeed, xSpeed);
      double newMoveMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (curMoveMagnitude != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / curMoveMagnitude);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(newMoveDirection, curMoveDirection);
      if (angleDif < 0.45*Math.PI) {
        curMoveDirection = SwerveUtils.StepTowardsCircular(curMoveDirection, newMoveDirection, directionSlewRate * elapsedTime);
        curMoveMagnitude = magnitudeLimiter.calculate(newMoveMagnitude);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (curMoveMagnitude > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          curMoveMagnitude = magnitudeLimiter.calculate(0.0);
        }
        else {
          curMoveDirection = SwerveUtils.WrapAngle(curMoveDirection + Math.PI);
          curMoveMagnitude = magnitudeLimiter.calculate(newMoveMagnitude);
        }
      }
      else {
        curMoveDirection = SwerveUtils.StepTowardsCircular(curMoveDirection, newMoveDirection, directionSlewRate * elapsedTime);
        curMoveMagnitude = magnitudeLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      // Convert back from polar to XY
      xSpeed = curMoveMagnitude * Math.cos(curMoveDirection);
      ySpeed = curMoveMagnitude * Math.sin(curMoveDirection);

      turningSpeed = m_rotLimiter.calculate(turningSpeed);
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, GYRO.getRotation2d());
    }
    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    setModuleStates(moduleStates, true);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void brake() {
    FRONT_LEFT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    FRONT_RIGHT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);;
    REAR_LEFT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);;
    REAR_RIGHT.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean optimize) {
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
