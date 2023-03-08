// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_CONSTANTS;

public class Lights extends SubsystemBase {
  private DigitalOutput redLED;
  private DigitalOutput greenLED;
  private DigitalOutput blueLED;
  private Timer timer;
  private double amplitude;

  /** Creates a new Lights. */
  public Lights() {
    redLED = new DigitalOutput(LED_CONSTANTS.RED_LED_PORT);
    greenLED = new DigitalOutput(LED_CONSTANTS.GREEN_LED_PORT);
    blueLED = new DigitalOutput(LED_CONSTANTS.BLUE_LED_PORT);

    redLED.setPWMRate(LED_CONSTANTS.PWM_RATE);
    greenLED.setPWMRate(LED_CONSTANTS.PWM_RATE);
    blueLED.setPWMRate(LED_CONSTANTS.PWM_RATE);

    redLED.enablePWM(0);
    greenLED.enablePWM(0);
    blueLED.enablePWM(0);

    timer = new Timer();
    timer.start();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Time", timer.get());
    SmartDashboard.putNumber("Amplitude", .9 * (1 + Math.sin(timer.get())) / 2 + .01);
  }

  public void breathe() {
    redLED.updateDutyCycle(0);
    double time = timer.get();
    amplitude = .9 * (1 + Math.sin(time / LED_CONSTANTS.FREQUENCY)) / 2 + .1;
    greenLED.updateDutyCycle(amplitude);
  }

  public void blink() {
    greenLED.updateDutyCycle(0);
    double time = timer.get();
    amplitude = (int)(time / LED_CONSTANTS.FREQUENCY) % 2 == 0 ? 1 : 0;
    redLED.updateDutyCycle(amplitude);
  }
}
