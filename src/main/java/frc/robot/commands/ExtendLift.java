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

public class ExtendLift extends Command {
  Timer timer;
  double speed = .5;
  double duration = 2;
  /**
   * Add your docs here.
   */
  public ExtendLift() {
    super();
    requires(Robot.lift);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.lift.runLiftMotor(speed);
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
    Robot.lift.runLiftMotor(0);
  }

  /**
   * Called when the command is forced to stop.
   * 
   * Stop the motor.
   */
  @Override
  protected void interrupted() {
    Robot.lift.runLiftMotor(0);
  }
}
