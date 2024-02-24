// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class Arm extends SubsystemBase {
  MotorController motor ;
  MotorController motor2; 
  
  /** Creates a new Climber. */
  public Arm() {
    motor = new CANSparkMax(ArmConstants.MOTOR_CHANNEL, MotorType.kBrushless);
    motor2 = new CANSparkMax(ArmConstants.MOTOR2_CHANNEL, MotorType.kBrushless);
  }
  public void run (double speed){
    motor.set(speed);
    motor2.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
