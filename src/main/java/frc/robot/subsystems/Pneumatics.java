/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  Compressor compressor;
  DoubleSolenoid leftSolenoid;
  DoubleSolenoid rightSolenoid;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Pneumatics() {
    compressor = new Compressor();
    leftSolenoid = new DoubleSolenoid(RobotMap.leftSolenoidForward, RobotMap.leftSolenoidBackward);
    rightSolenoid = new DoubleSolenoid(RobotMap.rightSolenoidForward, RobotMap.rightSolenoidBackward);
  }

  public void turnOnCompressor() {
    compressor.start();
  }

  public void turnOffCompressor() {
    compressor.stop();
  }

  public void extend() {
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
