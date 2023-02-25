// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
  public Solenoid solenoid;

  /** Creates a new Grabber. */
  public Grabber() {
    //TODO: Fix solenoid constructor.
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.SOLENOID_PORT);
  }
  
  public void toggle() {
    solenoid.toggle();
  }
}
