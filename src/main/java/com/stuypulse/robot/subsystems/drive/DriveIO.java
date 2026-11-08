/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        Distance leftDistance = Meters.zero();
        Distance rightDistance = Meters.zero();

        Rotation2d gyroYaw = Rotation2d.ZERO;
    }

    public default void updateInputs(DriveIOInputs inputs) {}

    public default void setLeft(double output) {}

    public default void setRight(double output) {}
}
