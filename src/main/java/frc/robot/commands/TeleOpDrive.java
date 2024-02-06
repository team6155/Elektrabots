// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Drivetrain;
import frc.utils.SwerveUtils;

/**
 * Drive the robot according to input from the controller.
 */
public class TeleOpDrive extends Command {
  private final Drivetrain DRIVETRAIN;
  private final ADXRS450_Gyro GYRO;
  private final Supplier<Double> X_SPEED_INPUT, Y_SPEED_INPUT, TURNING_SPEED_INPUT;
  private final Supplier<Boolean> BOOST;
  private final Supplier<Boolean> FIELD_ORIENTED;
  private boolean rateLimit;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //private final PIDController PID;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param controller The xbox controller used to drive the robot.
   */
  public TeleOpDrive(Drivetrain drivetrain, ADXRS450_Gyro gyro, Supplier<Double> xSpeedInput,
      Supplier<Double> ySpeedInput, Supplier<Double> turningSpeedInput,
      Supplier<Boolean> boost, Supplier<Boolean> fieldOriented, boolean rateLimit) {
    DRIVETRAIN = drivetrain;
    GYRO = gyro;
    X_SPEED_INPUT = xSpeedInput;
    Y_SPEED_INPUT = ySpeedInput;
    TURNING_SPEED_INPUT = turningSpeedInput;
    BOOST = boost;
    FIELD_ORIENTED = fieldOriented;
    this.rateLimit = rateLimit;

    //PID = new PIDController(1, .1, .1);

    addRequirements(DRIVETRAIN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Gyro", GYRO.getAngle());
    double xSpeed = X_SPEED_INPUT.get();
    double ySpeed = Y_SPEED_INPUT.get();
    double turningSpeed = TURNING_SPEED_INPUT.get();

    xSpeed = Math.abs(xSpeed) > InputConstants.DEADBAND ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > InputConstants.DEADBAND ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > InputConstants.DEADBAND ? turningSpeed : 0;

    // This section of code lifted directly from REVRobotics template.
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeed = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeed = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(turningSpeed);
    }

    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    turningSpeed *= DriveConstants.MAX_ANGULAR_SPEED;

    if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
      DRIVETRAIN.idle = true;
    }
    else {
      DRIVETRAIN.idle = false;
    }
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
    if (FIELD_ORIENTED.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds, GYRO.getRotation2d().rotateBy(DriveConstants.STARTING_ROTATION)
      );
    }

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    if (BOOST.get()) {
      DRIVETRAIN.setModuleStates(moduleStates, true, 1);
    }
    else {
      DRIVETRAIN.setModuleStates(moduleStates, true, DriveConstants.SPEED_RATIO);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    DRIVETRAIN.setModuleStates(moduleStates, false, DriveConstants.SPEED_RATIO);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double measurement() {
    return GYRO.getRotation2d().getRadians();
  }
}
