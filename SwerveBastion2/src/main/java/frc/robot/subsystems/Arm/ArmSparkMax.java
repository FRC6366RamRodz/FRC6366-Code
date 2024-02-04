// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class ArmSparkMax implements ArmIO {
    private static CANSparkMax UpperArm;
    private static CANSparkMax LowerArm;
    private static CANSparkMax LeftIntake;
    private static CANSparkMax RightIntake;
    private static SparkPIDController upperArmPid;
    private static SparkPIDController lowerArmPid;
    private static CANcoder UpperCoder;
    private static CANcoder LowerCoder;
    private final Solenoid Wrist;
    private final Solenoid ClawMode;
    private final Solenoid ArmBrakeU;
    private final Solenoid ArmBrakeL;
    private SparkLimitSwitch ArmUSwitch;
    private SparkLimitSwitch ArmLSwitch;
    private SparkLimitSwitch intakeSwitchl;
    private SparkLimitSwitch intakeSwithc2;

    public ArmSparkMax() {
        UpperArm.clearFaults();
        UpperArm.restoreFactoryDefaults();

        LowerArm.clearFaults();
        LowerArm.restoreFactoryDefaults();

        UpperArm = new CANSparkMax(5, MotorType.kBrushless);
        LowerArm = new CANSparkMax(6, MotorType.kBrushless);

        UpperArm.setInverted(false);
        LowerArm.setInverted(false);

        UpperCoder = new CANcoder(5);
        LowerCoder = new CANcoder(6);

        UpperArm.burnFlash();
        LowerArm.burnFlash();

        ArmBrakeL = new Solenoid(PneumaticsModuleType.REVPH, 0);
        ArmBrakeU = new Solenoid(PneumaticsModuleType.REVPH, 1);
        Wrist = new Solenoid(PneumaticsModuleType.REVPH, 3);
        ClawMode = new Solenoid(PneumaticsModuleType.REVPH, 2);

        LeftIntake = new CANSparkMax(7, MotorType.kBrushless);
        RightIntake = new CANSparkMax(8, MotorType.kBrushless);

        ArmUSwitch = UpperArm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        ArmLSwitch = UpperArm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        ArmUSwitch.enableLimitSwitch(true);
        ArmLSwitch.enableLimitSwitch(true);
        intakeSwitchl = LeftIntake.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        intakeSwithc2 = RightIntake.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        intakeSwitchl.enableLimitSwitch(true);
        intakeSwithc2.enableLimitSwitch(true);

        LeftIntake.setControlFramePeriodMs(500);
        RightIntake.setControlFramePeriodMs(500);

        upperArmPid = UpperArm.getPIDController();
        lowerArmPid = LowerArm.getPIDController();

        upperArmPid.setP(6e-5);
        upperArmPid.setI(0);
        upperArmPid.setD(0);
        upperArmPid.setFF(0.000015);
        upperArmPid.setOutputRange(-1, 01);

        lowerArmPid.setP(6e-5);
        lowerArmPid.setI(0);
        lowerArmPid.setD(0);
        lowerArmPid.setFF(0.000015);
        lowerArmPid.setOutputRange(-1, 01);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.LowerCoderPosition = LowerCoder.getAbsolutePosition().getValueAsDouble();
        inputs.UpperCoderPosition = UpperCoder.getAbsolutePosition().getValueAsDouble();
        inputs.BrakeL = ArmBrakeL.get();
        inputs.BrakeU = ArmBrakeU.get();

        double wristset;
        if(Wrist.get()) {
            wristset = 90;
        } else {
            wristset = 10;
        }
        inputs.IntakePosition = wristset;
    }


    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, boolean Intake, boolean Lbrake, boolean Ubrake, boolean IntakeMode, double IntakeSpeed) {
        ArmBrakeL.set(Lbrake);
        ArmBrakeU.set(Ubrake);
        Wrist.set(Intake);
        ClawMode.set(IntakeMode);
        LeftIntake.set(-IntakeSpeed);
        RightIntake.set(-IntakeSpeed);

        double MaxSpd = 5000;

        upperArmPid.setReference(upperSpeed*5000, ControlType.kVelocity);
        lowerArmPid.setReference(lowerSpeed*MaxSpd, ControlType.kVelocity);
    }

    public double getUArm() {
        return UpperArm.get();
    }

    public double getLarm() {
        return LowerArm.get();
    }

}
