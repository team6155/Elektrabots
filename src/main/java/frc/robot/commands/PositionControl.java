/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.utility.ColorInfo;

/**
 * Command for the Position Control part of the competition.
 * 
 * The robot must spin the control panel to a specific color.
 */
public class PositionControl extends Command {
  private Color expectedColor;
  private Color currentColor;

  // TODO: Assign command to a button.
  /**
   * Constructor for Position Control command.
   */
  public PositionControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.controlPanel);
  }

  /**
   * Runs once at the start of the command.
   * 
   * Read game data to know what color to stop on. Set the initial currentColor.
   */
  @Override
  protected void initialize() {
    // TODO: Make sure game data is valid.
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.equalsIgnoreCase("b")) {
      expectedColor = ColorInfo.BLUE; 
    }
    if (gameData.equalsIgnoreCase("r")) {
      expectedColor = ColorInfo.RED;
    }
    if (gameData.equalsIgnoreCase("g")) {
      expectedColor = ColorInfo.GREEN;
    }
    if (gameData.equalsIgnoreCase("y")) {
      expectedColor = ColorInfo.YELLOW;
    }
    if(expectedColor != null) {
      currentColor = ColorInfo.findClosestColor(Robot.controlPanel.readColor());
    }
  }

  /**
   * Called repeatedly until the command is finished.
   * 
   * Reading the current color and starting the motor.
   */
  @Override
  protected void execute() {
    currentColor = ColorInfo.findClosestColor(Robot.controlPanel.readColor());
    Robot.controlPanel.runMotor(.5);
  }

  /**
   * Check if the command is finished running.
   * 
   * Returns true if the current color is our expected color.
   */
  @Override
  protected boolean isFinished() {
    if (expectedColor == null || expectedColor.equals(currentColor)) {
      return true;
    }
    return false;
  }

  /**
   * Called once when the command is finished.
   * 
   * Stop the motor.
   */
  @Override
  protected void end() {
    Robot.controlPanel.runMotor(0);
  }

  /**
   * Called when the command is forced to stop.
   * 
   * Stop the motor.
   */
  @Override
  protected void interrupted() {
    Robot.controlPanel.runMotor(0);
  }
}
