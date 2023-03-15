// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberWrist;

public class AutonomousGrabber extends CommandBase {
  private final Grabber GRABBER;
  private final GrabberWrist GRABBER_WRIST;
  private final Timer timer;

  /** Creates a new AutonomousGrabber. */
  public AutonomousGrabber(Grabber grabber, GrabberWrist grabberWrist) {
    GRABBER = grabber;
    GRABBER_WRIST = grabberWrist;
    timer = new Timer();
    addRequirements(GRABBER);
    addRequirements(GRABBER_WRIST);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 2) {
      GRABBER_WRIST.run(1);
    }
    else if (timer.get() < 3) {
      GRABBER_WRIST.run(0);
      GRABBER.solenoid.set(Value.kForward);
    }
    else {
      GRABBER_WRIST.run(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    GRABBER_WRIST.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 5;
  }
}
