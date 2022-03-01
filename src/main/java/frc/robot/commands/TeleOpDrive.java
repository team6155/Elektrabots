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
    drivetrain.drive(-controller.getLeftY(), controller.getLeftX(), -controller.getRightX());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
