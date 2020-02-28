/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.RunScoopMotor;

/**
 * Add your docs here.
 */
public class ScoopMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public SpeedController motor;
  private double maximumSpeed;

  public ScoopMotor() {
    motor = new PWMVictorSPX(RobotMap.SCOOP_MOTOR);
    maximumSpeed = .5;
    motor.setInverted(true);
  }

  public void run(double speed) {
    motor.set(speed * maximumSpeed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RunScoopMotor(Robot.oi.OPERATOR_CONTROLLER));
  }
}
