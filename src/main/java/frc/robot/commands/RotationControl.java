/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.utility.ColorInfo;
import frc.robot.utility.Direction;

public class RotationControl extends Command {
  Color currentColor;
  Color previousColor;
  HashMap<Color, Integer> count;
  int highestCount;
  public RotationControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.controlPanel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    count = new HashMap<Color, Integer>();
    count.put(ColorInfo.BLUE, 0);
    count.put(ColorInfo.RED, 0);
    count.put(ColorInfo.YELLOW, 0);
    count.put(ColorInfo.GREEN, 0);
    previousColor = null;
    highestCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.controlPanel.runMotor(Direction.FORWARD);
    currentColor = ColorInfo.findClosestColor(Robot.controlPanel.readColor());
    if(!currentColor.equals(previousColor)){
      int currentCount = count.get(currentColor) + 1;
      count.put(currentColor, currentCount);
      if(currentCount > highestCount){
        highestCount = currentCount;
      }
    }
    previousColor = currentColor;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return highestCount > 7 && highestCount < 10;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.controlPanel.runMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.controlPanel.runMotor(Direction.STOP);
  }
}
