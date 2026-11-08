/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        Rotation2d yaw = Rotation2d.ZERO;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}
