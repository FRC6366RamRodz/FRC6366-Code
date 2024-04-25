// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** North charleston pre intake post shooter update. */
public class ShooterV2Hardware implements ShooterIO {

  // F_ is falcon500s K_ is Kraken x60s, N_ is Neo1.1s, f_ is 550/flares.
  public static TalonFX F_ArmMotor = new TalonFX(5);
  public static TalonFX K_topShooter = new TalonFX(6);
  public static TalonFX K_bottomShooter = new TalonFX(7);
  public static CANSparkMax N_Intake = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax N_Handler = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax f_Feeder = new CANSparkMax(7, MotorType.kBrushless);
  public static CANSparkMax N_Climber = new CANSparkMax(8, MotorType.kBrushless);
  public double armResetCount;

  // Encoders
  public static CANcoder ArmEncoder = new CANcoder(5);
  public static StatusSignal<Double> absolutePosition;
  public static StatusSignal<Double> TopVelocity;
  public static StatusSignal<Double> BottomVelocity;

  // limit Switch
  public SparkLimitSwitch HandlerSwitch = N_Handler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  public ShooterV2Hardware() {
    N_Climber.restoreFactoryDefaults();
    N_Climber.setIdleMode(IdleMode.kBrake);
    N_Climber.burnFlash();

    // Neo Stuff
    N_Intake.restoreFactoryDefaults();
    N_Handler.restoreFactoryDefaults();
    f_Feeder.restoreFactoryDefaults();

    N_Intake.clearFaults();
    N_Handler.clearFaults();
    f_Feeder.clearFaults();

    HandlerSwitch.enableLimitSwitch(true);

    N_Intake.setIdleMode(IdleMode.kCoast);
    N_Handler.setIdleMode(IdleMode.kBrake);
    f_Feeder.setIdleMode(IdleMode.kCoast);

    N_Intake.enableVoltageCompensation(12);
    N_Handler.enableVoltageCompensation(12);
    f_Feeder.enableVoltageCompensation(12);

    N_Intake.setInverted(true);
    f_Feeder.setInverted(true);

    N_Handler.setSmartCurrentLimit(80);
    N_Intake.setSmartCurrentLimit(80);
    f_Feeder.setSmartCurrentLimit(80);

    N_Handler.setOpenLoopRampRate(0.5);
    f_Feeder.setOpenLoopRampRate(0.5);

    N_Intake.burnFlash();
    N_Handler.burnFlash();
    f_Feeder.burnFlash();

    // TalonFX stuff
    var shooterConfig = new TalonFXConfiguration();
    //config
    shooterConfig.CurrentLimits.StatorCurrentLimit = 45;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.Slot0.kV = 0.118; //0.12 means apply 12V for a Target Velocity of 100 RPS or 6000 RPM.
    shooterConfig.Slot0.kP = 0.13;
    shooterConfig.Slot0.kI = 0.0;
    shooterConfig.Slot0.kD = 0.0;
    shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    K_topShooter.getConfigurator().apply(shooterConfig);
    K_bottomShooter.getConfigurator().apply(shooterConfig);

    var angleConfig = new TalonFXConfiguration();
    //config
    angleConfig.CurrentLimits.StatorCurrentLimit = 40;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    angleConfig.Slot0.kS = 0.55;//0.28
    angleConfig.Slot0.kG = 0.3;//0.4
    angleConfig.Slot0.kV = 0.0;
    angleConfig.Slot0.kP = 130.0;//75
    angleConfig.Slot0.kI = 0.0;
    angleConfig.Slot0.kD = 0.0;//0,75
    angleConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    angleConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    angleConfig.MotionMagic.MotionMagicAcceleration = 180; // 80 rps cruise velocity
    angleConfig.MotionMagic.MotionMagicCruiseVelocity = 90; // 160 rps/s acceleration (0.5 seconds)
    angleConfig.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    angleConfig.Feedback.SensorToMechanismRatio = 135;
    angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
    angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.143;

    F_ArmMotor.getConfigurator().apply(new TalonFXConfiguration());
    F_ArmMotor.getConfigurator().apply(angleConfig);

    absolutePosition = ArmEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(250, absolutePosition);
    ArmEncoder.optimizeBusUtilization();

    armResetCount = 0.0;
    
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(absolutePosition);

    inputs.TopVelocity = K_topShooter.getVelocity().getValueAsDouble() * 60;
    inputs.BottomVelocity = K_bottomShooter.getVelocity().getValueAsDouble() * 60;

    inputs.HandlerVelocity = N_Handler.getEncoder().getVelocity();
    inputs.feederVelocity = f_Feeder.getEncoder().getVelocity();

    inputs.anglePosition = absolutePosition.getValueAsDouble() * 360;
    inputs.angleVelocity = F_ArmMotor.getVelocity().getValueAsDouble()*60;

    inputs.intakeLimit = HandlerSwitch.isPressed();

    inputs.ArmResetCount = armResetCount;
    inputs.intakeAmps = N_Intake.getOutputCurrent();
  }

  @Override
  public void setMotors(
      double TopVelocity,
      double BottomVelocity,
      double HandlerVelocity,
      double anglePosition,
      double intakeVelocity,
      double feederVelocity,
      boolean limitOff,
      double climb) {

    // Arm Calculations
    //double ArmVolts = AngleFeedForward.calculate(Units.degreesToRadians(anglePosition), 0) + MathUtil.clamp(AnglePID.calculate(absolutePosition.getValueAsDouble() * 360, anglePosition), -4, 12);
    //F_ArmMotor.setVoltage(ArmVolts);
    F_ArmMotor.setControl(new MotionMagicVoltage(new Rotation2d(Units.degreesToRadians(anglePosition)).getRotations()).withSlot(0));
    
    if (F_ArmMotor.getPosition().getValueAsDouble() > new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).minus(new Rotation2d(Units.degreesToRadians(0.45))).getRotations() && F_ArmMotor.getPosition().getValueAsDouble() < new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).plus(new Rotation2d(Units.degreesToRadians(0.45))).getRotations()) {

    } else {
      F_ArmMotor.setPosition(new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).getRotations());
      armResetCount += 1;
    }

    K_bottomShooter.setControl(new VelocityVoltage(TopVelocity/60).withSlot(0));
    K_topShooter.setControl(new VelocityVoltage(BottomVelocity/60).withSlot(0));

    HandlerSwitch.enableLimitSwitch(!limitOff);
    N_Handler.set(HandlerVelocity);
   f_Feeder.set(feederVelocity);
    
    if (HandlerSwitch.isPressed()) {
       N_Intake.set(0);
    } else {
    N_Intake.set(intakeVelocity);
    }

    N_Climber.set(climb);
  }
}
