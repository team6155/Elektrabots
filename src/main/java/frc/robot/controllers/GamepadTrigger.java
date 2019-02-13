/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class GamepadTrigger extends Button {
    private final Joystick JOYSTICK;
    private final int AXIS;

    public GamepadTrigger(Joystick joystick, int axis) {
        this.JOYSTICK = joystick;
        this.AXIS = axis;
    }

    public boolean get() {
        return JOYSTICK.getRawAxis(AXIS) > 0;
    }
}
