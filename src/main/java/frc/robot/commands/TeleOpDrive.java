// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot according to input from the controller.
 */
public class TeleOpDrive extends Command {
  private final Drivetrain DRIVETRAIN;
  private final Supplier<Double> X_SPEED_INPUT, Y_SPEED_INPUT, TURNING_SPEED_INPUT;
  private final Supplier<Boolean> FIELD_ORIENTED;
  private boolean rateLimit;
  
  /**
   * Constructor for the Drive command
   */
  public TeleOpDrive(Drivetrain drivetrain, Supplier<Double> xSpeedInput,
      Supplier<Double> ySpeedInput, Supplier<Double> turningSpeedInput,
      Supplier<Boolean> fieldOriented, boolean rateLimit) {
    DRIVETRAIN = drivetrain;
    X_SPEED_INPUT = xSpeedInput;
    Y_SPEED_INPUT = ySpeedInput;
    TURNING_SPEED_INPUT = turningSpeedInput;
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
    double xSpeed = X_SPEED_INPUT.get();
    double ySpeed = Y_SPEED_INPUT.get();
    double turningSpeed = TURNING_SPEED_INPUT.get();

    xSpeed = Math.abs(xSpeed) > InputConstants.DEADBAND ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > InputConstants.DEADBAND ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > InputConstants.DEADBAND ? turningSpeed : 0;

    DRIVETRAIN.drive(xSpeed, ySpeed, turningSpeed, FIELD_ORIENTED.get(), rateLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVETRAIN.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
