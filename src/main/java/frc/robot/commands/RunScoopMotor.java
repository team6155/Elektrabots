/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.controllers.Gamepad;

/**
 * Add your docs here.
 */
public class RunScoopMotor extends InstantCommand {
  Gamepad controller;

  /**
   * Add your docs here.
   */
  public RunScoopMotor(Gamepad controller) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.scoopMotor);
    this.controller = controller;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double speed = 0;
    double forwardsSpeed = controller.getRightTrigger();
    double backwardsSpeed = -controller.getLeftTrigger();
    if (forwardsSpeed > 0 && backwardsSpeed == 0)
      speed = forwardsSpeed;
    if (backwardsSpeed < 0 && forwardsSpeed == 0)
      speed = backwardsSpeed;
    Robot.scoopMotor.run(speed);
  }
}
