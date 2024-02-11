// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class ShooterV2Hardware {

    // F_ is falcon500s K_ is Kraken x60s, N_ is Neo1.1s, f_ is 550/flares.
    public static TalonFX F_ArmMotor = new TalonFX(5);
    public static TalonFX K_topShooter = new TalonFX(6);
    public static TalonFX K_bottomShooter = new TalonFX(7);
    public static CANSparkMax N_Intake = new CANSparkMax(1, MotorType.kBrushless);
    public static CANSparkMax N_Handler = new CANSparkMax(2, MotorType.kBrushless);
    public static CANSparkMax f_Feeder = new CANSparkMax(3, MotorType.kBrushless);

    //Controll Loops for shooter wheels
    public static SimpleMotorFeedforward TopFeedForward;
    public static SimpleMotorFeedforward BottomFeedForward;
    public static PIDController TopPID;
    public static PIDController BottomPID;

    //Controll Loops for Angle motor
    public static ArmFeedforward AngleFeedForward;
    public static PIDController AnglePID;

    //Encoders
    public static CANcoder ArmEncoder = new CANcoder(5);

    //limit Switch
    public SparkLimitSwitch HandlerSwitch;

    public ShooterV2Hardware() {
        
        //Neo Stuff
        N_Intake.restoreFactoryDefaults();
        N_Handler.restoreFactoryDefaults();
        f_Feeder.restoreFactoryDefaults();

        N_Intake.clearFaults();
        N_Handler.clearFaults();
        f_Feeder.clearFaults();

            //lazy motor
        N_Intake.setControlFramePeriodMs(500);

        HandlerSwitch.enableLimitSwitch(true);

        N_Intake.setIdleMode(IdleMode.kCoast);
        N_Handler.setIdleMode(IdleMode.kCoast);
        f_Feeder.setIdleMode(IdleMode.kCoast);

        N_Intake.enableVoltageCompensation(12);
        N_Handler.enableVoltageCompensation(12);
        f_Feeder.enableVoltageCompensation(12);

        N_Intake.burnFlash();
        N_Handler.burnFlash();
        f_Feeder.burnFlash();

        //TalonFX stuff
        var shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.StatorCurrentLimit = 35;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        K_topShooter.getConfigurator().apply(shooterConfig);
        K_bottomShooter.getConfigurator().apply(shooterConfig);

        var angleConfig = new TalonFXConfiguration();
        angleConfig.CurrentLimits.StatorCurrentLimit = 40;
        angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        F_ArmMotor.getConfigurator().apply(angleConfig);


        //shooter stuff
        double SHks = 0.0;
        double SHkv = 0.0;
        TopFeedForward = new SimpleMotorFeedforward(SHks, SHkv);
        BottomFeedForward = new SimpleMotorFeedforward(SHks, SHkv);

        double SHp = 0.0;
        double SHi = 0.0;
        double SHd = 0.0;
        TopPID = new PIDController(SHp, SHi, SHd);

        //angle stuff
        double ARs = 0.0;
        double ARg = 0.0;
        double ARv = 0.0;
        AngleFeedForward = new ArmFeedforward(ARs, ARg, ARv);      
    }
}
