// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

/** Original Alpha style hardware, as parts hadnt arived yet */
public class ShooterV1Hardware implements ShooterIO {
  public TalonFX angleMotor = new TalonFX(5);
  public CANcoder angleEncoder = new CANcoder(5);
  public PIDController anglePID = new PIDController(0.2, 0, 0);
  public CANSparkMax topShooter = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax bottomShooter = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax FeedRoller = new CANSparkMax(7, MotorType.kBrushless);
  public CANSparkMax sideRoller = new CANSparkMax(8, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);
  public SparkLimitSwitch feeedSwitch;
  public SparkPIDController topShooterController;
  public SparkPIDController bottomShooterController;
  public SparkPIDController SideRollerController;
  // alt pid stuff
  /// *

  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;
  private SimpleMotorFeedforward topFEED;
  private SimpleMotorFeedforward bottomFEED;
  private PIDController topPID;
  private PIDController bottomPID;

  // */

  public ShooterV1Hardware() {

    var angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleMotor.getConfigurator().apply(angleConfig);

    bottomShooter.restoreFactoryDefaults();
    topShooter.restoreFactoryDefaults();

    topShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    FeedRoller.setIdleMode(IdleMode.kBrake);
    sideRoller.setIdleMode(IdleMode.kCoast);

    sideRoller.setInverted(true);

    intake.setControlFramePeriodMs(150);

    FeedRoller.setSmartCurrentLimit(20);
    sideRoller.setSmartCurrentLimit(20);

    feeedSwitch = FeedRoller.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    feeedSwitch.enableLimitSwitch(true);

    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();

    bottomShooter.setSmartCurrentLimit(35);
    topShooter.setSmartCurrentLimit(35);

    topShooterController = topShooter.getPIDController();
    bottomShooterController = bottomShooter.getPIDController();

    topShooterController.setFeedbackDevice(topShooter.getEncoder());
    bottomShooterController.setFeedbackDevice(bottomShooter.getEncoder());

    topShooterController.setP(6e-5);
    topShooterController.setI(0);
    topShooterController.setD(0);
    topShooterController.setIZone(0);
    topShooterController.setFF(0.00015);
    topShooterController.setOutputRange(-1, 1);

    bottomShooterController.setP(6e-5);
    bottomShooterController.setI(0);
    bottomShooterController.setD(0);
    bottomShooterController.setIZone(0);
    bottomShooterController.setFF(0.00015);
    bottomShooterController.setOutputRange(-1, 1);

    SideRollerController = sideRoller.getPIDController();

    SideRollerController.setP(6e-5);
    bottomShooterController.setI(0);
    bottomShooterController.setD(0);
    bottomShooterController.setFF(0.000015);
    bottomShooterController.setOutputRange(-1, 1);

    anglePID.enableContinuousInput(-360, 360);

    topShooter.burnFlash();
    bottomShooter.burnFlash();
    FeedRoller.burnFlash();
    sideRoller.burnFlash();
    intake.burnFlash();

    // alternate pid options
    /// *
    topFEED = new SimpleMotorFeedforward(0.103, 0.000178);
    bottomFEED = new SimpleMotorFeedforward(0.103, 0.000178);
    topPID = new PIDController(0.0, 0, 0.00000);
    bottomPID = new PIDController(0.0, 0, 0.00000);
    topEncoder = topShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();
    // */
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.TopVelocity =
        topShooter
            .getEncoder()
            .getVelocity(); // neos have a kv of 473 giving a max theoretical of 5676
    inputs.BottomVelocity = bottomShooter.getEncoder().getVelocity();
    inputs.anglePosition =
        Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.angleVelocity = (angleEncoder.getVelocity().getValueAsDouble()) * 60;
    inputs.HandlerVelocity = FeedRoller.getEncoder().getVelocity();
    inputs.intakeVelocity = intake.getEncoder().getVelocity();
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

    // topShooterController.setReference(TopVelocity, ControlType.kVelocity);
    // bottomShooterController.setReference(BottomVelocity, ControlType.kVelocity);

    double ShooterAngle =
        anglePID.calculate(
            Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble()),
            anglePosition);
    angleMotor.setControl(
        new VoltageOut(MathUtil.clamp(MathUtil.applyDeadband(-ShooterAngle, 1), -12, 12)));

    FeedRoller.set(-HandlerVelocity);
    intake.set(-intakeVelocity);

    SideRollerController.setReference(feederVelocity, ControlType.kVelocity);

    // alternate PID stuff if rev never works
    /// *
    double topPercent =
        MathUtil.applyDeadband(
            MathUtil.clamp(
                topPID.calculate(topEncoder.getVelocity(), TopVelocity)
                    + topFEED.calculate(TopVelocity),
                -1,
                1),
            0.1);
    double bottomPercent =
        MathUtil.applyDeadband(
            MathUtil.clamp(
                bottomPID.calculate(bottomEncoder.getVelocity(), BottomVelocity)
                    + bottomFEED.calculate(BottomVelocity),
                -1,
                1),
            0.1);

    topShooter.set(topPercent);
    bottomShooter.set(bottomPercent);
    // */
  }
}
