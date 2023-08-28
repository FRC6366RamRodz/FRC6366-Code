// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveHWIntr implements DriveIo {
    private static final double GEAR_RATIO = 6.0;
    private static final double TICKS_PER_REV = 2048;

    private static CANSparkMax LEFT_FRONT_MOTOR;
    private static CANSparkMax RIGHT_FRONT_MOTOR;
    private static CANSparkMax LEFT_REAR_MOTOR;
    public static CANSparkMax RIGHT_REAR_MOTOR;

    private RelativeEncoder LEFT_FRONT_ENCODER;
    private RelativeEncoder RIGHT_FRONT_ENCODER;

    public DriveHWIntr() {
        LEFT_FRONT_MOTOR = new CANSparkMax(1, MotorType.kBrushless);
        RIGHT_FRONT_MOTOR = new CANSparkMax(2, MotorType.kBrushless);
        LEFT_REAR_MOTOR = new CANSparkMax(3, MotorType.kBrushless);
        RIGHT_REAR_MOTOR = new CANSparkMax(4, MotorType.kBrushless);

        LEFT_FRONT_MOTOR.enableVoltageCompensation(12);
        LEFT_REAR_MOTOR.enableVoltageCompensation(12);
        RIGHT_FRONT_MOTOR.enableVoltageCompensation(12);
        RIGHT_REAR_MOTOR.enableVoltageCompensation(12);

        LEFT_REAR_MOTOR.follow(LEFT_FRONT_MOTOR);
        RIGHT_REAR_MOTOR.follow(RIGHT_FRONT_MOTOR);

        LEFT_FRONT_MOTOR.setInverted(false);
        RIGHT_FRONT_MOTOR.setInverted(false);
        LEFT_REAR_MOTOR.setInverted(false);
        RIGHT_REAR_MOTOR.setInverted(false);

        LEFT_FRONT_MOTOR.setSmartCurrentLimit(40);
        RIGHT_FRONT_MOTOR.setSmartCurrentLimit(40);
        LEFT_REAR_MOTOR.setSmartCurrentLimit(40);
        RIGHT_REAR_MOTOR.setSmartCurrentLimit(40);

        LEFT_FRONT_ENCODER = LEFT_FRONT_MOTOR.getEncoder();
        RIGHT_FRONT_ENCODER = RIGHT_FRONT_MOTOR.getEncoder();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(-LEFT_FRONT_ENCODER.getPosition()/TICKS_PER_REV/GEAR_RATIO);
        inputs.rightPostitionRad = Units.rotationsToRadians(-RIGHT_FRONT_ENCODER.getPosition()/TICKS_PER_REV/GEAR_RATIO);

        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(-LEFT_FRONT_ENCODER.getVelocity()*10/TICKS_PER_REV/GEAR_RATIO);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(-RIGHT_FRONT_ENCODER.getVelocity()*10/TICKS_PER_REV/GEAR_RATIO);
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        LEFT_FRONT_MOTOR.set(leftVolts / 12.0);
        RIGHT_FRONT_MOTOR.set(rightVolts / 12.0);
    }

}