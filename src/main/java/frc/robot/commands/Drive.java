/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Drive the robot according the the input from the xbox controller.
 */
public class Drive extends InstantCommand {
  XboxController controller;

  /**
   * Constructor for Drive command.
   * 
   * @param controller Xbox Controller used to control the robot.
   */
  public Drive(XboxController controller) {
    super();
    requires(Robot.driveTrain);
    this.controller = controller;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double forwardsSpeed = normalizeInput(controller.getY(Hand.kLeft));
    double sidewaysSpeed = normalizeInput(controller.getX(Hand.kLeft));
    double rotationSpeed = normalizeInput(controller.getX(Hand.kRight));

    Robot.driveTrain.drive(forwardsSpeed, sidewaysSpeed, rotationSpeed);
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
