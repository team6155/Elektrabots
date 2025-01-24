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
  private final double min = -15;
  private final double max = 185;
  private final double threshold = (max - min) * .25;
  private final double offset = 180;
  
  /** Creates a new Climber. */
  public Arm() {
    leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_CHANNEL, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_CHANNEL, MotorType.kBrushless);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);
    rightMotor.setSmartCurrentLimit(40);
    leftMotor.setSmartCurrentLimit(40);
    encoder = leftMotor.getEncoder();
  }

  public void run (double speed, boolean constrain){
    double constrainedSpeed = speed;
    double current = getOffsetEncoder();
    if (constrain ){
      if (current > (max - threshold) && speed > 0){
        constrainedSpeed *= (max - current)/threshold;
      }
      if (current < (min + threshold) && speed < 0){
        constrainedSpeed *= (current - min)/threshold;
      }
    }
    leftMotor.set(constrainedSpeed);
    rightMotor.set(constrainedSpeed);
  }

  private double getOffsetEncoder() {
    return encoder.getPosition() + offset;
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Encoder", getOffsetEncoder());
  }
}
