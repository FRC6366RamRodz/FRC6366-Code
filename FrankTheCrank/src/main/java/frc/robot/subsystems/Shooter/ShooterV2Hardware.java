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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterV2Hardware implements ShooterIO {

  // F_ is falcon500s K_ is Kraken x60s, N_ is Neo1.1s, f_ is 550/flares.
  public static TalonFX F_ArmMotor = new TalonFX(5);
  public static TalonFX K_topShooter = new TalonFX(6);
  public static TalonFX K_bottomShooter = new TalonFX(7);
  public static CANSparkMax N_Intake = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax N_Handler = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax f_Feeder = new CANSparkMax(7, MotorType.kBrushless);

  // Controll Loops for Angle motor
  public static ArmFeedforward AngleFeedForward;
  public static PIDController AnglePID;

  // Encoders
  public static CANcoder ArmEncoder = new CANcoder(5);
  public static StatusSignal<Double> absolutePosition;
  public static StatusSignal<Double> TopVelocity;
  public static StatusSignal<Double> BottomVelocity;

  // limit Switch
  public SparkLimitSwitch HandlerSwitch = N_Handler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  public ShooterV2Hardware() {

    // Neo Stuff
    N_Intake.restoreFactoryDefaults();
    N_Handler.restoreFactoryDefaults();
    f_Feeder.restoreFactoryDefaults();

    N_Intake.clearFaults();
    N_Handler.clearFaults();
    f_Feeder.clearFaults();

    // lazy motor
    N_Intake.setControlFramePeriodMs(450);
    f_Feeder.setControlFramePeriodMs(100);

    HandlerSwitch.enableLimitSwitch(true);

    N_Intake.setIdleMode(IdleMode.kCoast);
    N_Handler.setIdleMode(IdleMode.kBrake);
    f_Feeder.setIdleMode(IdleMode.kBrake);

    N_Intake.enableVoltageCompensation(12);
    N_Handler.enableVoltageCompensation(12);
    f_Feeder.enableVoltageCompensation(12);

    N_Intake.setInverted(true);

    N_Intake.burnFlash();
    N_Handler.burnFlash();
    f_Feeder.burnFlash();

    // TalonFX stuff
    var shooterConfig = new TalonFXConfiguration();
    //config
    shooterConfig.CurrentLimits.StatorCurrentLimit = 35;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.Slot0.kV = 0.12; //0.12 means apply 12V for a Target Velocity of 100 RPS or 6000 RPM.
    shooterConfig.Slot0.kP = 0.1;
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
    angleConfig.Slot0.kS = 0.0;
    angleConfig.Slot0.kV = 0.0;
    angleConfig.Slot0.kP = 0.0;
    angleConfig.Slot0.kI = 0.0;
    angleConfig.Slot0.kD = 0.0;
    angleConfig.MotionMagic.MotionMagicAcceleration = 160; // 80 rps cruise velocity
    angleConfig.MotionMagic.MotionMagicCruiseVelocity = 80; // 160 rps/s acceleration (0.5 seconds)
    angleConfig.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    angleConfig.Feedback.SensorToMechanismRatio = 100;
    angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
    angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1388;

    F_ArmMotor.getConfigurator().apply(new TalonFXConfiguration());
    F_ArmMotor.getConfigurator().apply(angleConfig);

    absolutePosition = ArmEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(250, absolutePosition);
    ArmEncoder.optimizeBusUtilization();

    // angle stuff
    double ARs = 0.0;
    double ARg = 0.4;
    double ARv = 0.0;
    AngleFeedForward = new ArmFeedforward(ARs, ARg, ARv);


    double ARp = 0.15;//0.15
    double ARi = 0.18;//0.2
    double ARd = 0.000;
    AnglePID = new PIDController(ARp, ARi, ARd);

    AnglePID.enableContinuousInput(-180, 180);
    
    AnglePID.setTolerance(2);
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
  }

  @Override
  public void setMotors(
      double TopVelocity,
      double BottomVelocity,
      double HandlerVelocity,
      double anglePosition,
      double intakeVelocity,
      double feederVelocity,
      boolean limitOff) {

    // Arm Calculations
    //double ArmVolts = AngleFeedForward.calculate(Units.degreesToRadians(anglePosition), 0) + MathUtil.clamp(AnglePID.calculate(absolutePosition.getValueAsDouble() * 360, anglePosition), -4, 12);
    //F_ArmMotor.setVoltage(ArmVolts);
    F_ArmMotor.setControl(new MotionMagicVoltage(new Rotation2d(Units.degreesToRadians(anglePosition)).getRotations()).withSlot(0).withOverrideBrakeDurNeutral(true));
    
    if (F_ArmMotor.getPosition().getValueAsDouble() > new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).minus(new Rotation2d(Units.degreesToRadians(0.5))).getRotations() && F_ArmMotor.getPosition().getValueAsDouble() < new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).plus(new Rotation2d(Units.degreesToRadians(0.5))).getRotations()) {

    } else {
      F_ArmMotor.setPosition(new Rotation2d(Units.rotationsToRadians(ArmEncoder.getAbsolutePosition().getValueAsDouble())).getRotations());
    }

    K_bottomShooter.setControl(new VelocityVoltage(TopVelocity).withSlot(0));
    K_topShooter.setControl(new VelocityVoltage(BottomVelocity).withSlot(0));

    HandlerSwitch.enableLimitSwitch(!limitOff);
    N_Handler.set(HandlerVelocity);
    f_Feeder.set(feederVelocity);
    
    if (HandlerSwitch.isPressed()) {
       N_Intake.set(0);
    } else {
    N_Intake.set(intakeVelocity);
    }
  }
}
