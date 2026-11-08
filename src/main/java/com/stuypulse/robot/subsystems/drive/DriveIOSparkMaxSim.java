/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DifferentialDrivetrainSim;
import org.wpilib.system.RobotController;

public class DriveIOSparkMaxSim implements DriveIO {
    private final DifferentialDrivetrainSim sim =
            new DifferentialDrivetrainSim(
                    DCMotor.getNEO(2),
                    7.2,
                    0.5,
                    60.0,
                    Units.inchesToMeters(3.0),
                    Units.inchesToMeters(24.0),
                    null);

    private double leftThrottle = 0;
    private double rightThrottle = 0;

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        sim.setInputs(
                leftThrottle * RobotController.getBatteryVoltage(),
                rightThrottle * RobotController.getBatteryVoltage());
        sim.update(0.02);

        inputs.leftDistance = Meters.of(sim.getLeftPosition());
        inputs.rightDistance = Meters.of(sim.getRightPosition());

        inputs.gyroYaw = sim.getHeading();
    }

    @Override
    public void setLeft(double output) {
        leftThrottle = output;
    }

    @Override
    public void setRight(double output) {
        rightThrottle = output;
    }
}
