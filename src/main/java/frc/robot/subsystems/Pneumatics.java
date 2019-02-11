package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  private Compressor compressor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;

  /**
   * Constructor for the Pneumatics subsystem.
   */
  public Pneumatics() {
    compressor = new Compressor();
    leftSolenoid =
        new DoubleSolenoid(RobotMap.LEFT_SOLENOID_FORWARD, RobotMap.LEFT_SOLENOID_BACKWARD);
    rightSolenoid =
        new DoubleSolenoid(RobotMap.RIGHT_SOLENOID_FORWARD, RobotMap.RIGHT_SOLENOID_BACKWARD);
  }

  /**
   * Start the compressor.
   */
  public void turnOnCompressor() {
    compressor.start();
  }

  /**
   * Stop the compressor.
   */
  public void turnOffCompressor() {
    compressor.stop();
  }

  /**
   * Extend both arms of the robot.
   */
  public void extend() {
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Retract both arms of the robot.
   */
  public void retract() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  // Set the default command of the subsystem.
  // This command will run whenever the subsystem is not being used by another command.
  @Override
  public void initDefaultCommand() {
  }
}
