// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
  public DoubleSolenoid solenoid;

  /** Creates a new Grabber. */
  public Grabber() {
    //TODO: Fix solenoid constructor.
    solenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      GrabberConstants.FORWARD_SOLENOID_PORT,
      GrabberConstants.REVERSE_SOLENOID_PORT
    );
    solenoid.set(Value.kReverse);
  }
  
  public void toggle() {
    solenoid.toggle();
  }
  
  @Override
  public void periodic() {}
}
