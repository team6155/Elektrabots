// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot according to input from the controller.
 */
public class TeleOpDrive extends CommandBase {
  private final double BASE_SPEED_MULTIPLE = .5;

  private final Drivetrain DRIVETRAIN;
  private final XboxController CONTROLLER;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param controller The xbox controller used to drive the robot.
   */
  public TeleOpDrive(Drivetrain drivetrain, XboxController controller) {
    this.DRIVETRAIN = drivetrain;
    this.CONTROLLER = controller;
    addRequirements(this.DRIVETRAIN);
  }

  @Override
  public void execute() {
    double forwardsSpeed = adjustInput(-CONTROLLER.getLeftY());
    double sidewaysSpeed = adjustInput(CONTROLLER.getLeftX());
    double rotationalSpeed = adjustInput(-CONTROLLER.getRightX());
    DRIVETRAIN.drive(forwardsSpeed, sidewaysSpeed, rotationalSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Adjust the input from the joystick for finer control.
   * <p>
   * Multiply the input by a multiple dependent on the controller's right trigger and square the input.
   * @param input The input given by the joystick. [-1.0 .. 1.0].
   * @return The normalized input 
   */
  private double adjustInput(double input) {
    double sign = input >= 1 ? 1 : -1;
    double speedMultiple = CONTROLLER.getRightTriggerAxis() * (1 - BASE_SPEED_MULTIPLE) + BASE_SPEED_MULTIPLE;
    double squaredInput = input * input;
    return sign * squaredInput * speedMultiple;
  }
}
