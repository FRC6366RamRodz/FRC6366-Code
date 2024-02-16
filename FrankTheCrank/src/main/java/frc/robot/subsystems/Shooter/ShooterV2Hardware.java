// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  // Controll Loops for shooter wheels
  public static SimpleMotorFeedforward TopFeedForward;
  public static SimpleMotorFeedforward BottomFeedForward;
  public static PIDController TopPID;
  public static PIDController BottomPID;

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
    shooterConfig.CurrentLimits.StatorCurrentLimit = 35;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    K_topShooter.getConfigurator().apply(shooterConfig);
    K_bottomShooter.getConfigurator().apply(shooterConfig);

    var angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.StatorCurrentLimit = 40;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    F_ArmMotor.getConfigurator().apply(angleConfig);

    absolutePosition = ArmEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(250, absolutePosition);
    ArmEncoder.optimizeBusUtilization();

    // shooter stuff
    double SHks = 0.94;
    double SHkv = 0.00161;
    TopFeedForward = new SimpleMotorFeedforward(SHks, SHkv);
    BottomFeedForward = new SimpleMotorFeedforward(SHks, SHkv);

    double SHp = 0.0001;
    double SHi = 0.0;
    double SHd = 0.00;
    TopPID = new PIDController(SHp, SHi, SHd);
    BottomPID = new PIDController(SHp, SHi, SHd);

    // angle stuff
    double ARs = 0.0;
    double ARg = 0.4;
    double ARv = 0.0;
    AngleFeedForward = new ArmFeedforward(ARs, ARg, ARv);


    double ARp = 0.11;
    double ARi = 0.0;
    double ARd = 0.0006;
    AnglePID = new PIDController(ARp, ARi, ARd);

    AnglePID.enableContinuousInput(-180, 180);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.TopVelocity = K_topShooter.getVelocity().getValueAsDouble() * 60;
    inputs.BottomVelocity = K_bottomShooter.getVelocity().getValueAsDouble() * 60;

    inputs.HandlerVelocity = N_Handler.getEncoder().getVelocity();
    inputs.feederVelocity = f_Feeder.getEncoder().getVelocity();

    inputs.anglePosition = absolutePosition.getValueAsDouble() * 360;
    inputs.angleVelocity = ArmEncoder.getVelocity().getValueAsDouble() * 60;
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
    double ArmVolts = AngleFeedForward.calculate(Units.degreesToRadians(anglePosition), 0) + MathUtil.clamp(AnglePID.calculate(absolutePosition.getValueAsDouble() * 360, anglePosition), -5, 12);
    F_ArmMotor.setVoltage(ArmVolts);
    

    // Shooter
    double topVolts = TopFeedForward.calculate(TopVelocity) + TopPID.calculate(K_topShooter.getVelocity().getValueAsDouble()*60,TopVelocity);
    double bottomVolts = BottomFeedForward.calculate(BottomVelocity) + BottomPID.calculate(K_bottomShooter.getVelocity().getValueAsDouble()*60,BottomVelocity);
    K_topShooter.setVoltage(topVolts);
    K_bottomShooter.setVoltage(bottomVolts);

    HandlerSwitch.enableLimitSwitch(!limitOff);
    N_Handler.set(HandlerVelocity);
    f_Feeder.set(feederVelocity);

    N_Intake.set(intakeVelocity);
  }
}
