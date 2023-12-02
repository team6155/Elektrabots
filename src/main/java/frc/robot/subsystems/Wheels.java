// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheels extends SubsystemBase {
  // tpdp: change channel numbers
public WPI_VictorSPX frontRightWheel = new WPI_VictorSPX(6);
public WPI_VictorSPX frontLeftWheel = new WPI_VictorSPX(3);
public WPI_VictorSPX rearRightWheel = new WPI_VictorSPX(7);
public WPI_VictorSPX rearLeftWheel = new WPI_VictorSPX(2);
  /** Creates a new Wheels. */
  public Wheels() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
