// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class ShooterV1Hardware implements ShooterIO {
    public TalonFX angleMotor = new TalonFX(5);
    public CANSparkMax topShooter = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax bottomShooter = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax Feed1 = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax Feed2 = new CANSparkMax(4, MotorType.kBrushless);
    public CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
    public SparkLimitSwitch feeedSwitch;

    public ShooterV1Hardware() {
        var angleConfig = new TalonFXConfiguration();
        angleConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        angleMotor.getConfigurator().apply(angleConfig);
        
        topShooter.setSmartCurrentLimit(35);
        bottomShooter.setSmartCurrentLimit(35);

        intake.setControlFramePeriodMs(150);

        Feed1.setSmartCurrentLimit(20);
        Feed2.setSmartCurrentLimit(20);
        
        feeedSwitch = Feed1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        feeedSwitch.enableLimitSwitch(true);
    }
}
