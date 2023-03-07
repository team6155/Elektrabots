// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_CONSTANTS;

public class Lights extends SubsystemBase {
  private PWM redLED;
  private PWM greenLED;
  private PWM blueLED;
  private Timer timer;
  private int amplitude;

  /** Creates a new Lights. */
  public Lights() {
    redLED = new PWM(LED_CONSTANTS.RED_LED_PORT);
    greenLED = new PWM(LED_CONSTANTS.GREEN_LED_PORT);
    blueLED = new PWM(LED_CONSTANTS.BLUE_LED_PORT);
    timer = new Timer();
    timer.start();
  }
  
  @Override
  public void periodic() {
    //SmartDashboard.putString("Port", redLED.readString());
    SmartDashboard.putString("LED", "" + amplitude);
  }

  public void breathe() {
    redLED.setRaw(0);
    double time = timer.get();
    amplitude = (int)((4096 - 1) * (1 + Math.sin(time)) / 2) + 1;
    greenLED.setRaw(amplitude);
  }

  public void blink() {
    greenLED.setRaw(0);
    double time = timer.get();
    amplitude = (int)time % 2 == 0 ? 1 : 4096;
    redLED.setRaw(amplitude);
  }
}
