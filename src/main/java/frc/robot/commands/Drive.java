/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;

public class Drive extends Command {
  XboxController controller;

  /**
   * Constructor for Drive command
   * 
   * @param controller Xbox Controller used to control the robot.
   */
  public Drive(XboxController controller) {
    requires(Robot.driveTrain);
    this.controller = controller;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forwardsSpeed = normalizeInput(controller.getY(Hand.kLeft));
    double sidewaysSpeed = normalizeInput(controller.getX(Hand.kLeft));
    double rotationSpeed = normalizeInput(controller.getX(Hand.kRight));

    Robot.driveTrain.drive(forwardsSpeed, sidewaysSpeed, rotationSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }


  /**
   * Normalize the input from the controller to give finer control at low speeds and to discard tiny movements of the stick.
   * <p>Square the input while keeping the positive or negative sign. If the squared input is < 0.02, discard it.
   * 
   * @param stickInput Operator input from the xbox controller joystick.
   * @return Normalized speed.
   */
  private double normalizeInput(double stickInput) {
    int sign = stickInput >= 0 ? 1 : -1;

    stickInput *= stickInput;
    if (stickInput < 0.02) stickInput = 0;
    stickInput *= sign;

    return stickInput;
  }
}
