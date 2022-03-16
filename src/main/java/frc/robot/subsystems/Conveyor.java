// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The robot's Conveyor subsystem.
 * <p>
 * This is responsible for picking up and shooting the balls.
 */
public class Conveyor extends SubsystemBase {
  private final double INTAKE_MAX_SPEED = .5;
  private final double BELT_MAX_SPEED = .5;
  private final double SHOOTING_MAX_SPEED = .5;

  private final MotorController INTAKE_MOTOR;
  private final MotorController BELT_MOTOR;
  private final MotorController SHOOTING_MOTOR;
  
  /** Creates a new Conveyor subsystem. */
  public Conveyor() {
    INTAKE_MOTOR = new PWMVictorSPX(Constants.INTAKE_CHANNEL);
    BELT_MOTOR = new PWMVictorSPX(Constants.BELT_CHANNEL);
    SHOOTING_MOTOR = new PWMVictorSPX(Constants.SHOOTING_CHANNEL);
  }

  /**
   * Run the intake motor at a given speed.
   * @param speed The desired speed of the motor.
   */
  public void runIntakeMotor(double speed) {
    INTAKE_MOTOR.set(speed * INTAKE_MAX_SPEED);
  }

  /**
   * Run the belt motor at a given speed.
   * @param speed The desired speed of the motor.
   */
  public void runBeltMotor(double speed) {
    BELT_MOTOR.set(speed * BELT_MAX_SPEED);
  }

  /**
   * Run the shooting motor at a given speed.
   * @param speed The desired speed of the motor.
   */
  public void runShootingMotor(double speed) {
    SHOOTING_MOTOR.set(speed * SHOOTING_MAX_SPEED);
  }
}
