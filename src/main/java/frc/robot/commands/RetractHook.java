/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RetractHook extends Command {
  Timer timer;
  double speed = .5;
  double duration = 2;
  /**
   * Add your docs here.
   */
  public RetractHook() {
    super();
    requires(Robot.hook);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.hook.runHookMotor(speed);
    timer = new Timer();
    timer.start();
  }

  @Override
  protected boolean isFinished() {
    // TODO Auto-generated method stub
    double time = timer.get();
    if(time > duration){
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
    Robot.hook.runHookMotor(0);
  }

  /**
   * Called when the command is forced to stop.
   * 
   * Stop the motor.
   */
  @Override
  protected void interrupted() {
    Robot.hook.runHookMotor(0);
  }

}
