package frc.robot;

import frc.robot.commands.ChangeDirection;
import frc.robot.commands.ControlPanelManual;
import frc.robot.commands.ExtendControlPanel;
import frc.robot.commands.ExtendLift;
import frc.robot.commands.ExtendScoop;
import frc.robot.commands.PositionControl;
import frc.robot.commands.RotationControl;
import frc.robot.commands.UpdateSensors;
import frc.robot.controllers.Gamepad;
import frc.robot.subsystems.ControlPanel;
import frc.robot.utility.Direction;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public final Gamepad DRIVER_CONTROLLER;
  public final Gamepad OPERATOR_CONTROLLER;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    DRIVER_CONTROLLER = new Gamepad(RobotMap.DRIVER_GAMEPAD);
    OPERATOR_CONTROLLER = new Gamepad(RobotMap.OPERATOR_GAMEPAD);

    DRIVER_CONTROLLER.A_BUTTON.whenPressed(new ChangeDirection());

    OPERATOR_CONTROLLER.A_BUTTON.whenPressed(new ExtendScoop());
    OPERATOR_CONTROLLER.B_BUTTON.whileHeld(new UpdateSensors());
    OPERATOR_CONTROLLER.Y_BUTTON.whenPressed(new ExtendControlPanel());
    OPERATOR_CONTROLLER.D_PAD_RIGHT.whileHeld(new ControlPanelManual(Direction.FORWARD));
    OPERATOR_CONTROLLER.D_PAD_RIGHT.whenReleased(new ControlPanelManual(Direction.STOP));
    OPERATOR_CONTROLLER.D_PAD_LEFT.whileHeld(new ControlPanelManual(Direction.REVERSE));
    OPERATOR_CONTROLLER.D_PAD_LEFT.whenReleased(new ControlPanelManual(Direction.STOP));
    OPERATOR_CONTROLLER.START_BUTTON.whenPressed(new RotationControl());
    OPERATOR_CONTROLLER.BACK_BUTTON.whenPressed(new PositionControl());
    OPERATOR_CONTROLLER.D_PAD_UP.whileHeld(new ExtendLift(Direction.FORWARD));
    OPERATOR_CONTROLLER.D_PAD_UP.whenReleased(new ExtendLift(Direction.STOP));
    OPERATOR_CONTROLLER.D_PAD_DOWN.whileHeld(new ExtendLift(Direction.REVERSE));
    OPERATOR_CONTROLLER.D_PAD_DOWN.whenReleased(new ExtendLift(Direction.STOP));
  }
}
