/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.Auton;
import com.stuypulse.robot.commands.auton.AutonFactory;
import com.stuypulse.robot.commands.subsystems.DriveCommands;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.FieldConstants;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Camera;
import com.stuypulse.robot.subsystems.vision.VisionConstants.VisionMode;
import com.stuypulse.robot.subsystems.vision.VisionIO;
import com.stuypulse.robot.subsystems.vision.VisionIOLimelight;
import com.stuypulse.robot.subsystems.vision.VisionIOPhoton;
import com.stuypulse.robot.subsystems.vision.VisionIOPhotonSim;

import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.tunable.Tunables;

import java.util.EnumMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;

public class RobotContainer {

    // Gamepads
    private final CommandGamepad driver =
            new CommandGamepad(DriverConstants.Driver.DRIVER_INDEX); // model-agnostic controller
    // public final CommandGamepad operator = new CommandGamepad(GlobalPorts.Gamepad.OPERATOR);

    // Subsystems
    private final Vision vision;
    // Autons
    private final LoggedNetworkChooser<Auton> autonChooser = new LoggedNetworkChooser<>("Auton");

    // Robot container

    public RobotContainer() {
        EnumMap<Camera, VisionIO> visionIOMap = new EnumMap<>(Camera.class);
        switch (GlobalSettings.CURRENT_MODE) {
            case REAL -> {
                for (Camera camera : VisionConstants.Camera.values()) {
                    visionIOMap.put(
                            camera,
                            VisionConstants.VISION_MODE == VisionMode.LIMELIGHT
                                    ? new VisionIOLimelight(
                                            camera.getName(),
                                            camera.getRobotToCamera(),
                                            swerve::getRotation)
                                    : new VisionIOPhoton(camera.getName(), camera.getRobotToCamera()));
                }
            }

            case SIM -> {
                for (Camera camera : Camera.values()) {
                    visionIOMap.put(
                            camera,
                            new VisionIOPhotonSim(
                                    camera.getName(), camera.getRobotToCamera(), swerve::getPose));
                }
            }

            default -> {
                for (Camera camera : Camera.values()) {
                    visionIOMap.put(camera, new VisionIO() {});
                }
            }
        }

        configureLogging();
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        Tunables.publish("Field", FieldConstants.FIELD_2D);
    }

    /***************/
    /*** LOGGING ***/
    /***************/

    private void configureLogging() {}

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve, driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.dpadUp().onTrue(DriveCommands.resetHeading(swerve, Rotation2d.ZERO));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    private void configureAutons() {
        AutonFactory autonFactory = new AutonFactory();

        autonChooser.addDefault("Do Nothing", autonFactory.doNothingAuton());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return The command to run in autonomous
     */
    public Auton getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
