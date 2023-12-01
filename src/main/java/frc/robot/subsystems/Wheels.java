// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheels extends SubsystemBase {
  // tpdp: change channel numbers
public PWMVictorSPX frontRightWheel = new PWMVictorSPX(-1);
public PWMVictorSPX frontLeftWheel = new PWMVictorSPX(-1);
public PWMVictorSPX rearRightWheel = new PWMVictorSPX(-1);
public PWMVictorSPX rearLeftWheel = new PWMVictorSPX(-1);
  /** Creates a new Wheels. */
  public Wheels() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
