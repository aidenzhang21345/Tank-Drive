/************************ PROJECT ZIXI ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;

public class DriveIOSparkMax implements DriveIO {
    private final SparkMax leftLeader = new SparkMax(CANPort.CAN_D0, 0, MotorType.kBrushless);
    private final SparkMax leftFollower = new SparkMax(CANPort.CAN_D0, 1, MotorType.kBrushless);
    private final SparkMax rightLeader = new SparkMax(CANPort.CAN_D0, 2, MotorType.kBrushless);
    private final SparkMax rightFollower = new SparkMax(CANPort.CAN_D0, 3, MotorType.kBrushless);

    private final Pigeon2 gyro = new Pigeon2(0, new CANBus());

    private final SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    private final Signal<Double> leftPosition = leftLeader.getEncoder().getPosition();
    private final Signal<Double> rightPosition = rightLeader.getEncoder().getPosition();

    public DriveIOSparkMax() {
        leftLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftFollower.configure(
                config.follow(leftLeader, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightLeader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFollower.configure(
                config.follow(leftLeader, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftDistance =
                Meters.of(leftPosition.get() * DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
        inputs.rightDistance =
                Meters.of(rightPosition.get() * DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));

        inputs.gyroYaw = gyro.getRotation2d();
    }

    @Override
    public void setLeft(double output) {
        leftLeader.setThrottle(output);
    }

    @Override
    public void setRight(double output) {
        rightLeader.setThrottle(output);
    }
}
