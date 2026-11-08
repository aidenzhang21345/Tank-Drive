/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import com.stuypulse.robot.subsystems.vision.Vision.VisionConsumer;
import com.stuypulse.robot.util.PeriodicSubsystem;

import org.wpilib.drive.DifferentialDrive;
import org.wpilib.math.estimator.DifferentialDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends PeriodicSubsystem {
    private final DifferentialDrive drive;

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final DifferentialDrivePoseEstimator poseEstimator =
            new DifferentialDrivePoseEstimator(
                    inputs.gyroYaw,
                    inputs.leftDistance.in(Meters),
                    inputs.rightDistance.in(Meters),
                    Pose2d.ZERO);

    private final VisionConsumer visionConsumer = poseEstimator::addVisionMeasurement;

    public Drive(DriveIO io) {
        this.io = io;

        drive = new DifferentialDrive(this.io::setLeft, this.io::setRight);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Drivetrain", inputs);

        poseEstimator.update(
                inputs.gyroYaw, inputs.leftDistance.in(Meters), inputs.rightDistance.in(Meters));
    }

    @AutoLogOutput(key = "Drive/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
                inputs.gyroYaw,
                inputs.leftDistance.in(Meters),
                inputs.rightDistance.in(Meters),
                pose);
    }

    public void resetHeading(Rotation2d heading) {
        resetPose(new Pose2d(getPose().getTranslation(), heading));
    }

    public VisionConsumer getVisionConsumer() {
        return visionConsumer;
    }

    public void drive(double xVelocity, double zRotation, boolean squared) {
        drive.arcadeDrive(xVelocity, zRotation, squared);
    }
}
