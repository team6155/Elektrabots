package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.RunBelt;

/**
 * Subsystem for controlling the belt.
 * <p>
 * The belt is controlled by a motor which runs a belt to pick up the ball from
 * the ground, raise it up, and then shoot it out.
 */
public class Belt extends Subsystem {
  SpeedController motor;

  /**
   * Constructor for the Belt subsystem.
   */
  public Belt() {
    motor = new PWMVictorSPX(RobotMap.BELT_MOTOR);
  }

  /**
   * Run the belt at a given speed.
   * <p>
   * This method will set the speed of the motor which will cause the belt to run
   * in a loop, causing the ball to go up or down the robot.
   * 
   * @param speed The to spin the motor. Positive speed runs the belt such that
   *              the ball will move up the robot. Negative numbers will move the
   *              belt in the opposite direction.
   */
  public void run(double speed) {
    // Setting the speed of the motor actually runs the belt in the wrong direction,
    // so the speed is set to its negative. This causes the desired result.
    motor.set(-speed);
  }

  /**
   * Run the motor for a second to make sure it's working properly.
   */
  public void testMotor() {
    run(1);
    Timer.delay(1);
    run(0);
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
    setDefaultCommand(new RunBelt(Robot.oi.getController()));
  }
}
