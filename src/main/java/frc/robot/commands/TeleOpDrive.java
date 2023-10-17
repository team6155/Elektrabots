// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot according to input from the controller.
 */
public class TeleOpDrive extends CommandBase {
  private final Drivetrain DRIVETRAIN;
  private final Gyro GYRO;
  private final Supplier<Double> X_SPEED_INPUT, Y_SPEED_INPUT, TURNING_SPEED_INPUT;
  private final Supplier<Boolean> BOOST;
  private final Supplier<Boolean> FIELD_ORIENTED;

  //private final PIDController PID;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param controller The xbox controller used to drive the robot.
   */
  public TeleOpDrive(Drivetrain drivetrain, Gyro gyro, Supplier<Double> xSpeedInput,
      Supplier<Double> ySpeedInput, Supplier<Double> turningSpeedInput,
      Supplier<Boolean> boost, Supplier<Boolean> fieldOriented) {
    DRIVETRAIN = drivetrain;
    GYRO = gyro;
    X_SPEED_INPUT = xSpeedInput;
    Y_SPEED_INPUT = ySpeedInput;
    TURNING_SPEED_INPUT = turningSpeedInput;
    BOOST = boost;
    FIELD_ORIENTED = fieldOriented;

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
    double xSpeed = -X_SPEED_INPUT.get();
    double ySpeed = -Y_SPEED_INPUT.get();
    double turningSpeed = -TURNING_SPEED_INPUT.get();

    //turningSpeed = PID.calculate(measurement(), measurement() + turningSpeed);

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
    if (FIELD_ORIENTED.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds, GYRO.getRotation2d().rotateBy(DriveConstants.STARTING_ROTATION)
      );
    }

    SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

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
