/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class GuessColor extends InstantCommand {
  /**
   * Add your docs here.
   */

  private final Color BLUE = new Color(0, 0, 1);
  private final Color GREEN = new Color(0, 1, 0);
  private final Color RED = new Color(1, 0, 0);
  private final Color YELLOW = new Color(1, 1, 0);
  private final Color[] COLORS = {
    BLUE, GREEN, RED, YELLOW
  };
  
  public GuessColor() {
    super();
    requires(Robot.controlPanel);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Color color = Robot.controlPanel.readColor();
    SmartDashboard.putString("RGB", (int)(color.red * 100) + ", " + (int)(color.green * 100) + ", " + (int)(color.blue * 100));
    
    Color closest = findClosestColor(color);
    String output = "";
    if(closest.equals(BLUE)) {
      output = "Blue";
    }
    if(closest.equals(GREEN)) {
      output = "Green";
    }
    if(closest.equals(RED)) {
      output = "Red";
    }
    if(closest.equals(YELLOW)) {
      output = "Yellow";
    }
    SmartDashboard.putString("Closest Color", output);
  }

  private Color findClosestColor(Color color) {
    double minDistance = Double.MAX_VALUE;
    Color closest = null;
    for(Color templateColor : COLORS) {
      double distance = distance(color, templateColor);
      if(distance < minDistance) {
        minDistance = distance;
        closest = templateColor;
      }
    }
    return closest;
  }

  private double distance(Color color1, Color color2) {
    double redDistance = color1.red - color2.red;
    double greenDistance = color1.green - color2.green;
    double blueDistance = color1.blue - color2.blue;
    return Math.sqrt(redDistance * redDistance + greenDistance * greenDistance + blueDistance * blueDistance);
  }
}
