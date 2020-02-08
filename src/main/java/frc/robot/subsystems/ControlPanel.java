/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ControlPanel extends Subsystem {
  private DoubleSolenoid extender;
  private SpeedController motor;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ControlPanel() {
    extender = new DoubleSolenoid(RobotMap.CONTROL_PANEL_FORWARD, RobotMap.CONTROL_PANEL_BACKWARD);
    motor = new PWMVictorSPX(RobotMap.CONTROL_PANEL_MOTOR);
  }

  public void Extend() {
    extender.set(DoubleSolenoid.Value.kForward);
  }

  public void Retract() {
    extender.set(DoubleSolenoid.Value.kReverse);
  }

  public void RunMotor(double speed) {
    motor.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
