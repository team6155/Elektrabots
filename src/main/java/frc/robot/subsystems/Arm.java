// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor ;
  CANSparkMax rightMotor; 
  private final AbsoluteEncoder encoder; 
  
  /** Creates a new Climber. */
  public Arm() {
    leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_CHANNEL, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_CHANNEL, MotorType.kBrushless);
    leftMotor.setInverted(true);
    encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setInverted(true);
  }
  public void run (double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  public void run (double leftspeed, double rightspeed){
    leftMotor.set(leftspeed);
    rightMotor.set(rightspeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
