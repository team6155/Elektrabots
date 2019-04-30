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
 * Spin the belt to collect or launch the ball.
 */
public class RunBelt extends InstantCommand {
  Gamepad controller;

  /**
   * Constructor for RetractPusher Command.
   */
  public RunBelt(Gamepad controller) {
    super();
    requires(Robot.belt);
    this.controller = controller;
  }

  /**
   * The initialize method is called the first time this Command is run after
   * being started.
   * <p>
   * Call the belt subsystem's run method.
   */
  @Override
  protected void initialize() {
    double speed = 0;
    double forwardsSpeed = controller.getRightTrigger();
    double backwardsSpeed = controller.getLeftTrigger();
    if (forwardsSpeed > 0 && backwardsSpeed == 0)
      speed = forwardsSpeed;
    if (backwardsSpeed > 0 && forwardsSpeed == 0)
      speed = -backwardsSpeed;
    Robot.belt.run(speed);
  }

}
