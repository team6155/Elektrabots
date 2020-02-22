package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MecanumDriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.ScoopMotor;
import frc.robot.subsystems.ScoopPneumatics;
import frc.robot.subsystems.Hook;
import frc.robot.subsystems.Lift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //TODO: Reactivate camera
  public static DriveTrain driveTrain = new MecanumDriveTrain();
  public static Belt belt = new Belt();
  public static Pneumatics pneumatics = new Pneumatics();
  //public static Camera camera = new Camera();
  public static Hook hook = new Hook();
  public static Lift lift = new Lift();
  public static ControlPanel controlPanel = new ControlPanel();
  public static ScoopPneumatics scoopPneumatics = new ScoopPneumatics();
  public static ScoopMotor scoopMotor = new ScoopMotor();
  
  public static OI oi;

  /**
   * This function is run when the robot is first started up.
   */
  @Override
  public void robotInit() {
    oi = new OI();
  }

  /**
   * This function is called every robot packet, no matter the mode.
   * 
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically during Disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during Autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Operator Control
   * mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during Operator Control mode.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during Test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
