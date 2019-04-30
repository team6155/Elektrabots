package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem for controlling the pneumatic pusher on the robot.
 * <p>
 * The system is made up of two double solenoids controlling two arms on the
 * robot that can extend and retract as well as an air compressor.
 */
public class Pneumatics extends Subsystem {
  private Compressor compressor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid middleSolenoid;
  private DoubleSolenoid rightSolenoid;

  /**
   * Constructor for the Pneumatics subsystem.
   */
  public Pneumatics() {
    compressor = new Compressor();
    leftSolenoid = new DoubleSolenoid(RobotMap.LEFT_SOLENOID_FORWARD, RobotMap.LEFT_SOLENOID_BACKWARD);
    middleSolenoid = new DoubleSolenoid(RobotMap.MIDDLE_SOLENOID_FORWARD, RobotMap.MIDDLE_SOLENOID_BACKWARD);
    rightSolenoid = new DoubleSolenoid(RobotMap.RIGHT_SOLENOID_FORWARD, RobotMap.RIGHT_SOLENOID_BACKWARD);
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
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    middleSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Retract both arms of the robot.
   */
  public void retract() {
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    middleSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Initialize the default command for a subsystem. By default subsystems have no
   * default command, but if they do, the default command is set with this method.
   * It is called on all Subsystems by CommandBase in the users program after all
   * the Subsystems are created.
   * <p>
   * When the subsystem is not currently in use, it will call the command set in
   * this method.
   */
  @Override
  public void initDefaultCommand() {
    // The pneumatics system currently needs no default command.
  }
}
