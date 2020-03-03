/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ColorInfo {
    public static final Color BLUE = new Color(.01, .39, .65);
    public static final Color GREEN = new Color(0, .39, .21);
    public static final Color RED = new Color(.60, .11, .11);
    public static final Color YELLOW = new Color(.68, .60, .10);
    public static final Color[] COLORS = {BLUE, GREEN, RED, YELLOW};

    public static Color findClosestColor(Color color) {
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
    
      public static double distance(Color color1, Color color2) {
        double redDistance = color1.red - color2.red;
        double greenDistance = color1.green - color2.green;
        double blueDistance = color1.blue - color2.blue;
        return Math.sqrt(redDistance * redDistance + greenDistance * greenDistance + blueDistance * blueDistance);
      }
}
