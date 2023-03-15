// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class Lights extends SubsystemBase {
  private DigitalOutput redLED;
  private DigitalOutput greenLED;
  private DigitalOutput blueLED;
  private Timer timer;
  private double amplitude;
  private double safetyTime;

  /** Creates a new Lights. */
  public Lights() {
    redLED = new DigitalOutput(LEDConstants.RED_LED_PORT);
    greenLED = new DigitalOutput(LEDConstants.GREEN_LED_PORT);
    blueLED = new DigitalOutput(LEDConstants.BLUE_LED_PORT);

    redLED.setPWMRate(LEDConstants.PWM_RATE);
    greenLED.setPWMRate(LEDConstants.PWM_RATE);
    blueLED.setPWMRate(LEDConstants.PWM_RATE);

    redLED.enablePWM(0);
    greenLED.enablePWM(0);
    blueLED.enablePWM(0);

    timer = new Timer();
    timer.start();
  }
  
  @Override
  public void periodic() {}

  public void breathe(double red, double green, double blue) {
    double time = getTime() / LEDConstants.BREATHE_FREQUENCY;
    amplitude = .74 * (1 + Math.sin(getTime())) / 2 + .01;
    redLED.updateDutyCycle(amplitude * red);
    greenLED.updateDutyCycle(amplitude * green);
    blueLED.updateDutyCycle(amplitude * blue);
  }

  public void blink() {
    greenLED.updateDutyCycle(0);
    int time = (int)(getTime() / LEDConstants.BLINK_FREQUENCY);
    amplitude = (int)getTime() % 2 == 0 ? .75 : 0;
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      redLED.updateDutyCycle(amplitude);
      blueLED.updateDutyCycle(0);
    }
    else {
      blueLED.updateDutyCycle(amplitude);
      redLED.updateDutyCycle(0);
    }
  }

  public double getTime() {
    return timer.get();
  }
}
