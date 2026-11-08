package com.stuypulse.robot.commands.subsystems;

import com.stuypulse.robot.subsystems.drive.Drive;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;

public final class DriveCommands {
    private DriveCommands() {}

    public static Command joystickDrive(Drive drive, CommandGamepad driver) {
        return drive.runRepeatedly(() -> {
            drive.drive(driver.getLeftY(), driver.getRightX(), true);
        }).named("Joystick Drive");
    }
}
