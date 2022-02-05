package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;

public class RumbleGamepad extends CommandBase {

    private final Gamepad gamepad;

    public RumbleGamepad(GamepadEx gamepadex) {
        this.gamepad = gamepadex.gamepad;
    }

    public void execute() {
        gamepad.rumble(RobotConstants.RUMBLE_DURATION);
    }

}
