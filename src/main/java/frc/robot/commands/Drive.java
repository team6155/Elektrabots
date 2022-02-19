// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// TODO: Command is not called anywhere.
/**
 * Drive the robot according to input from the controller.
 */
public class Drive extends InstantCommand {
  Drivetrain drivetrain;
  double ySpeed;
  double xSpeed;
  double zRotation;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param ySpeed The desired speed forwards or backwards.
   * @param xSpeed The desired speed left or right.
   * @param zRotation The desired rotation clockwise or counter-clockwise.
   */
  public Drive(Drivetrain drivetrain, double ySpeed, double xSpeed, double zRotation) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  /**
   * The initial subroutine of a command. Called once when the command is initially scheduled.
   * <p>
   * Call the drivetrain subsystem's drive method.
   */
  @Override
  public void initialize() {
    drivetrain.drive(ySpeed, xSpeed, zRotation);
  }
}
