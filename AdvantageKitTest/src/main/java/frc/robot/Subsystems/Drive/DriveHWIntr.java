// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveHWIntr implements DriveIo {
    private static final double GEAR_RATIO = 6.0;
    private static final double TICKS_PER_REV = 2048;

    private static TalonFX LEFT_FRONT_MOTOR;
    private static TalonFX RIGHT_FRONT_MOTOR;
    private static TalonFX LEFT_REAR_MOTOR;
    public static TalonFX RIGHT_REAR_MOTOR;

    public DriveHWIntr() {
        LEFT_FRONT_MOTOR = new TalonFX(0);
        RIGHT_FRONT_MOTOR = new TalonFX(1);
        LEFT_REAR_MOTOR = new TalonFX(2);
        RIGHT_REAR_MOTOR = new TalonFX(4);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.statorCurrLimit.enable  = true;
        config.statorCurrLimit.currentLimit = 40;

        LEFT_FRONT_MOTOR.configAllSettings(config);
        RIGHT_FRONT_MOTOR.configAllSettings(config);

        LEFT_REAR_MOTOR.follow(LEFT_FRONT_MOTOR);
        RIGHT_REAR_MOTOR.follow(RIGHT_FRONT_MOTOR);

        LEFT_FRONT_MOTOR.setInverted(false);
        RIGHT_FRONT_MOTOR.setInverted(false);
        LEFT_REAR_MOTOR.setInverted(InvertType.FollowMaster);
        RIGHT_REAR_MOTOR.setInverted(InvertType.FollowMaster);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(LEFT_FRONT_MOTOR.getSelectedSensorPosition()/TICKS_PER_REV/GEAR_RATIO);
        inputs.rightPostitionRad = Units.rotationsToRadians(RIGHT_FRONT_MOTOR.getSelectedSensorPosition()/TICKS_PER_REV/GEAR_RATIO);

        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(LEFT_FRONT_MOTOR.getSelectedSensorVelocity()*10/TICKS_PER_REV/GEAR_RATIO);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(RIGHT_FRONT_MOTOR.getSelectedSensorVelocity()*10/TICKS_PER_REV/GEAR_RATIO);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        LEFT_FRONT_MOTOR.set(ControlMode.PercentOutput, leftVolts / 12.0);
        RIGHT_FRONT_MOTOR.set(ControlMode.PercentOutput, rightVolts / 12.0);
    }


}