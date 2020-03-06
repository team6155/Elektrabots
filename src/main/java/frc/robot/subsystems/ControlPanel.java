/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ControlPanel extends Subsystem {
  public DoubleSolenoid extender;
  public SpeedController motor;
  public ColorSensorV3 colorSensor;
  public Value direction = Value.kForward;
  private double MAX_SPEED = .3;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public ControlPanel() {
    extender = new DoubleSolenoid(RobotMap.CONTROL_PANEL_FORWARD, RobotMap.CONTROL_PANEL_BACKWARD);
    motor = new PWMVictorSPX(RobotMap.CONTROL_PANEL_MOTOR);
    colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR);
  }

  public void extend() {
    if(direction == Value.kForward) {
      direction = Value.kReverse;
    }
    else {
      direction = Value.kForward;
    }
    extender.set(direction);
  }

  public void runMotor(double speed) {
    motor.set(speed * MAX_SPEED);
  }

  public Color readColor() {
    return colorSensor.getColor();
  }
  
  public int getIR() {
    return colorSensor.getIR();
  }

  public int getProximity() {
    return colorSensor.getProximity();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new GuessColor());
  }
}
