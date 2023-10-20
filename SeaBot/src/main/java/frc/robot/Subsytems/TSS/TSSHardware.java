// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.TSS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Util.Constants.DT_PIDF;

/** Add your docs here. */
public class TSSHardware implements TssIO {
    private static TalonFX LF_MOTOR;
    private static TalonFX LR_MOTOR;
    private static TalonFX RF_MOTOR;
    private static TalonFX RR_MOTOR;
    private static CANSparkMax STF_MOTOR;
    private static CANSparkMax STR_MOTOR;
    private static Solenoid stinger;
    private static Solenoid Strafe;
    private static Solenoid Booper;

    public TSSHardware() {
        LF_MOTOR = new TalonFX(0);
        LR_MOTOR = new TalonFX(1);
        RF_MOTOR = new TalonFX(2);
        RR_MOTOR = new TalonFX(3);
        STF_MOTOR = new CANSparkMax(1, MotorType.kBrushless);
        STR_MOTOR = new CANSparkMax(2, MotorType.kBrushless);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.statorCurrLimit.enable = true;
        config.statorCurrLimit.currentLimit = 40;
        config.nominalOutputForward = 0;
        config.nominalOutputReverse = 0;
        config.peakOutputForward = 1;
        config.peakOutputReverse = -1;
        config.motionAcceleration = 6000.0;
        config.motionCruiseVelocity = 6000.0;

        LF_MOTOR.configAllSettings(config);
        RF_MOTOR.configAllSettings(config);

        LF_MOTOR.config_kP(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_PG,
            DT_PIDF.DT_TimeoutMs
        );
        LF_MOTOR.config_kI(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_IG,
            DT_PIDF.DT_TimeoutMs
        );
        LF_MOTOR.config_kD(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_DG,
            DT_PIDF.DT_TimeoutMs
        );
        LF_MOTOR.config_kF(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_F,
            DT_PIDF.DT_TimeoutMs
        );

        RF_MOTOR.config_kP(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_PG,
            DT_PIDF.DT_TimeoutMs
        );
        RF_MOTOR.config_kI(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_IG,
            DT_PIDF.DT_TimeoutMs
        );
        RF_MOTOR.config_kD(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_DG,
            DT_PIDF.DT_TimeoutMs
        );
        RF_MOTOR.config_kF(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_F,
            DT_PIDF.DT_TimeoutMs
        );

        

        LR_MOTOR.follow(LF_MOTOR);
        RR_MOTOR.follow(RF_MOTOR);

        LF_MOTOR.setInverted(true);
        RF_MOTOR.setInverted(false);
        LR_MOTOR.setInverted(InvertType.FollowMaster);
        RR_MOTOR.setInverted(InvertType.FollowMaster);

        STF_MOTOR.restoreFactoryDefaults();
        STR_MOTOR.restoreFactoryDefaults();

        STF_MOTOR.setInverted(false);
        STR_MOTOR.setInverted(false);

        STF_MOTOR.burnFlash();
        STR_MOTOR.burnFlash();

        Strafe = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        stinger = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        Booper = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    }

    @Override
    public void UpdateInputs(TssIOInputs inputs) {

    }

    @Override
    public void drive(double Left, double Right, double stingFront, double stingRear, boolean Strafe, boolean Sringer, boolean booper) {
        LF_MOTOR.set(ControlMode.PercentOutput, Left/12);
        RF_MOTOR.set(ControlMode.PercentOutput, Right/12);
        STF_MOTOR.set(stingFront);
        STR_MOTOR.set(stingRear);
        stinger.set(Sringer);
        TSSHardware.Strafe.set(Strafe);
        Booper.set(booper);
    }

    @Override
    public void autonomous(double left, double right, boolean booper) {
        LF_MOTOR.set(ControlMode.MotionMagic, left);
        RF_MOTOR.set(ControlMode.MotionMagic, right);
        stinger.set(false);
        TSSHardware.Strafe.set(false);
        Booper.set(booper);
    }
}
