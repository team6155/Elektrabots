/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.RunBelt;

/**
 * Add your docs here.
 */
public class Belt extends Subsystem {
  SpeedController motor;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Belt() {
    motor = new PWMVictorSPX(RobotMap.BELT_MOTOR);
  }

  public void run(double speed) {
    motor.set(-speed);
  }

  public void testMotor() {
    Timer timer = new Timer();
    motor.set(-1);
    timer.delay(1);
    motor.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RunBelt(Robot.oi.getController()));
  }
}
