// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  MotorController motorRight;
  MotorController motorLeft;

  /** Creates a new Shooter. */
  public Shooter() {
    motorRight = new CANSparkMax(ShooterConstants.MOTOR_CHANNEL_RIGHT, MotorType.kBrushless);
    motorLeft = new CANSparkMax(ShooterConstants.MOTOR_CHANNEL_LEFT, MotorType.kBrushless);
    motorRight.setInverted(true);
    motorLeft.setInverted(true);
  }

  public void run(double speed){
    motorRight.set(speed);
    motorLeft.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
