/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ScoopPneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  private Value direction;

  public ScoopPneumatics() {
    leftSolenoid = new DoubleSolenoid(RobotMap.SCOOP_LEFT_SOLENOID_REVERSE, RobotMap.SCOOP_LEFT_SOLENOID_FORWARD);
    rightSolenoid = new DoubleSolenoid(RobotMap.SCOOP_RIGHT_SOLENOID_REVERSE, RobotMap.SCOOP_RIGHT_SOLENOID_FORWARD);
    direction = Value.kForward;
  }
  
  public void extend() {
    if(direction == Value.kForward) {
      direction = Value.kReverse;
    }
    else {
      direction = Value.kForward;
    }
    leftSolenoid.set(direction);
    rightSolenoid.set(direction);
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
