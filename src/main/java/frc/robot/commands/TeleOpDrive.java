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
  double baseSpeedMultiple = .5;
  
  Drivetrain drivetrain;
  XboxController controller;
  
  /**
   * Constructor for the Drive command
   * @param drivetrain The drivetrain subsystem.
   * @param controller The xbox controller used to drive the robot.
   */
  public TeleOpDrive(Drivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(this.drivetrain);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double forwardsSpeed = adjustInput(-controller.getLeftY());
    double sidewaysSpeed = adjustInput(controller.getLeftX());
    double rotationalSpeed = adjustInput(-controller.getRightX());
    drivetrain.drive(forwardsSpeed, sidewaysSpeed, rotationalSpeed);
  }

  @Override
  public void end(boolean interrupted) {

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
    double speedMultiple = controller.getRightTriggerAxis() * (1 - baseSpeedMultiple) + baseSpeedMultiple;
    double squaredInput = input * input;
    return sign * squaredInput * speedMultiple;
  }
}
