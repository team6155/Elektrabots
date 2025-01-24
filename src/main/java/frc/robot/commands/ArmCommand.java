// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
  private final double limit = 1;
  private final Supplier<Double> speed ;
  private final Supplier<Boolean> upOverride;
  private final Supplier<Boolean> downOverride;
  private final Supplier<Boolean> overide ;
  private final Arm arm ;
  /** Creates a new ClimberCommand. */
  public ArmCommand(Arm arm, Supplier<Double> speed, Supplier<Boolean> upOverride, Supplier<Boolean> downOverride, Supplier<Boolean> overide) {
    addRequirements(arm);
    this.speed = speed ;
    this.arm = arm ;
    this.overide = overide ;
    this.upOverride = upOverride;
    this.downOverride = downOverride;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (overide.get()) {
      if (upOverride.get()) {
        arm.run(.2, false);
      }
      else if (downOverride.get()) {
        arm.run(-.2, false);
      }
      else {
        arm.run(0, false);
      }
    }
    else {
      arm.run(speed.get()*limit, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.run(0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
