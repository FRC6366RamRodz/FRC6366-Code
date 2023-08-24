// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Util.Constants;
import frc.robot.Util.Constants.AR_SET;

/** Add your docs here. */
public class ArmHwIntr implements ArmIO {

    private static CANSparkMax UpperArm;
    private static CANSparkMax LowerArm;
    private RelativeEncoder UpperArmEncoder;
    private RelativeEncoder lowerArmEncoder;
    private SparkMaxPIDController upperArmPID;
    private SparkMaxPIDController lowerArmPID;
    private static CANCoder UpperCoder;
    private static CANCoder LowerCoder;

    public ArmHwIntr() {
        UpperArm = new CANSparkMax(5, MotorType.kBrushless);
        LowerArm = new CANSparkMax(6, MotorType.kBrushless);

        UpperArm.setInverted(false);
        LowerArm.setInverted(false);

        UpperArmEncoder = UpperArm.getEncoder();
        lowerArmEncoder = LowerArm.getEncoder();

        UpperCoder = new CANCoder(5);
        LowerCoder = new CANCoder(6);

        upperArmPID.setP(AR_SET.kP);
        lowerArmPID.setP(AR_SET.kP);
        //set I
        upperArmPID.setI(AR_SET.kI);
        lowerArmPID.setI(AR_SET.kI);
        //set D
        upperArmPID.setD(AR_SET.kD);
        lowerArmPID.setD(AR_SET.kD);
        //set FF
        upperArmPID.setFF(AR_SET.kFF);
        lowerArmPID.setFF(AR_SET.kFF);
        //set Output range
        upperArmPID.setOutputRange(AR_SET.kMinOutput, AR_SET.kMaxOutput);
        lowerArmPID.setOutputRange(AR_SET.kMinOutput, AR_SET.kMaxOutput);

        UpperArm.burnFlash();
        LowerArm.burnFlash();
    }

    @Override
    public void updateInputs(ArmIoInputs inputs) {
        inputs.LowerCoderPosition = LowerCoder.getAbsolutePosition();
        inputs.UpperCoderPosition = UpperCoder.getAbsolutePosition();

        inputs.LowerMotorVelocity = lowerArmEncoder.getVelocity();
        inputs.UpperMotorVelocity = UpperArmEncoder.getVelocity();
    }

    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, double UsetPoint, double LsetPoint) {
        upperArmPID.setReference(upperSpeed*Constants.AR_SET.maxVel, CANSparkMax.ControlType.kVelocity);
        lowerArmPID.setReference(lowerSpeed*Constants.AR_SET.maxVel, CANSparkMax.ControlType.kVelocity);
    }


}
