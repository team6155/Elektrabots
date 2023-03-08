// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot according to input from the controller.
 */
public class TeleOpDrive extends CommandBase {
  private final Drivetrain DRIVETRAIN;
  private final Supplier<Double> X_SPEED_INPUT, Y_SPEED_INPUT, TURNING_SPEED_INPUT;
  private final SlewRateLimiter X_LIMITER, Y_LIMITER;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param controller The xbox controller used to drive the robot.
   */
  public TeleOpDrive(Drivetrain drivetrain,
      Supplier<Double> xSpeedInput, Supplier<Double> ySpeedInput, Supplier<Double> turningSpeedInput) {
    DRIVETRAIN = drivetrain;
    X_SPEED_INPUT = xSpeedInput;
    Y_SPEED_INPUT = ySpeedInput;
    TURNING_SPEED_INPUT = turningSpeedInput;
    X_LIMITER = new SlewRateLimiter(.9);
    Y_LIMITER = new SlewRateLimiter(.9);
    addRequirements(DRIVETRAIN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = X_LIMITER.calculate(X_SPEED_INPUT.get());
    double ySpeed = Y_LIMITER.calculate(Y_SPEED_INPUT.get());
    double turningSpeed = TURNING_SPEED_INPUT.get();

    xSpeed = Math.abs(xSpeed) > InputConstants.DEADBAND ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > InputConstants.DEADBAND ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > InputConstants.DEADBAND ? turningSpeed : 0;

    if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
      DRIVETRAIN.idle = true;
    }
    else {
      DRIVETRAIN.idle = false;
    }
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    DRIVETRAIN.setModuleStates(moduleStates, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    DRIVETRAIN.setModuleStates(moduleStates, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
