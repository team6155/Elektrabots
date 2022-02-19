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
  MotorController intakeMotor;
  MotorController shootingMotor;
  
  /** Creates a new Conveyor subsystem. */
  public Conveyor() {
    intakeMotor = new PWMVictorSPX(Constants.INTAKE_CHANNEL);
    shootingMotor = new PWMVictorSPX(Constants.SHOOTING_CHANNEL);
  }

  /**
   * Run the intake motor at a given speed.
   * @param speed The desired speed of the motor.
   */
  public void runIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Run the shooting motor at a given speed.
   * @param speed The desired speed of the motor.
   */
  public void runShootingMotor(double speed) {
    shootingMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
