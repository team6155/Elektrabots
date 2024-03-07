// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor ;
  CANSparkMax rightMotor; 
  private final RelativeEncoder encoder; 
  private final double min = -1;
  private final double max = -1; //TODO: figure out what this value actually is
  private final double threshold = -1;
  private final boolean constrain = false;
  
  /** Creates a new Climber. */
  public Arm() {
    leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_CHANNEL, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_CHANNEL, MotorType.kBrushless);
    rightMotor.setInverted(true);
    encoder = leftMotor.getEncoder();
  }

  public void run (double speed){
    double current = encoder.getPosition();
    if (constrain ){
      if (current > max - threshold){
        speed *= (max - current)/threshold;
      }
      if (current < min + threshold){
        speed *= (current - min)/threshold;
      }
    }
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder", encoder.getPosition());
  }
}
