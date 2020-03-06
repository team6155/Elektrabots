/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class GetSignal extends InstantCommand {
  /**
   * Add your docs here.
   */
  public GetSignal() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.controlPanel);
  }

  // Called once when the command executes
  // @Override
  // protected void initialize() {
  //   String gameData = DriverStation.getInstance().getGameSpecificMessage();
  //   if (gameData.equalsIgnoreCase("b")) {
  //     expectedColor = ColorInfo.BLUE; 
  //   }
  //   if (gameData.equalsIgnoreCase("r")) {
  //     expectedColor = ColorInfo.RED;
  //   }
  //   if (gameData.equalsIgnoreCase("g")) {
  //     expectedColor = ColorInfo.GREEN;
  //   }
  //   if (gameData.equalsIgnoreCase("y")) {
  //     expectedColor = ColorInfo.YELLOW;
  //   }
  //   if(expectedColor != null) {
  //     currentColor = ColorInfo.findClosestColor(Robot.controlPanel.readColor());
  //   }
  // }

}
