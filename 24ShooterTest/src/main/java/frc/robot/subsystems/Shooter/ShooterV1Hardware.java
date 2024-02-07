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
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterV1Hardware implements ShooterIO {
  public TalonFX angleMotor = new TalonFX(5);
  public CANcoder angleEncoder = new CANcoder(5);
  public PIDController anglePID = new PIDController(0.2, 0, 0);
  public SimpleMotorFeedforward angleFeedback = new SimpleMotorFeedforward(0.1, 0.13);
  public CANSparkMax topShooter = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax bottomShooter = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax FeedRoller = new CANSparkMax(7, MotorType.kBrushless);
  public CANSparkMax sideRoller = new CANSparkMax(8, MotorType.kBrushless);
  public CANSparkMax intake = new CANSparkMax(9, MotorType.kBrushless);
  public SparkLimitSwitch feeedSwitch;
  public SparkPIDController topShooterController;
  public SparkPIDController bottomShooterController;
  public SparkPIDController SideRollerController;

  public ShooterV1Hardware() {

    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();

    var angleConfig = new TalonFXConfiguration();
    angleConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleMotor.getConfigurator().apply(angleConfig);

    topShooter.setSmartCurrentLimit(80);
    topShooter.setInverted(false);
    bottomShooter.setSmartCurrentLimit(80);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    FeedRoller.setIdleMode(IdleMode.kBrake);
    sideRoller.setIdleMode(IdleMode.kCoast);

    intake.setControlFramePeriodMs(150);

    FeedRoller.setSmartCurrentLimit(20);
    sideRoller.setSmartCurrentLimit(20);

    feeedSwitch = FeedRoller.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    feeedSwitch.enableLimitSwitch(true);

    topShooterController = topShooter.getPIDController();
    bottomShooterController = bottomShooter.getPIDController();

    topShooterController.setP(0.001);
    topShooterController.setI(0);
    topShooterController.setD(0);
    topShooterController.setFF(0.005);
    topShooterController.setOutputRange(-1, 1);

    bottomShooterController.setP(0.001);
    bottomShooterController.setI(0);
    bottomShooterController.setD(0);
    bottomShooterController.setFF(0.005);
    bottomShooterController.setOutputRange(-1, 1);

    SideRollerController = sideRoller.getPIDController();

    SideRollerController.setP(6e-5);
    bottomShooterController.setI(0);
    bottomShooterController.setD(0);
    bottomShooterController.setFF(0.000015);
    bottomShooterController.setOutputRange(-1, 1);

    anglePID.enableContinuousInput(-360, 360);
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
    inputs.feederVelocity = FeedRoller.getEncoder().getVelocity();
    inputs.intakeVelocity = intake.getEncoder().getVelocity();
  }

  @Override
  public void setMotors(
      double TopVelocity,
      double BottomVelocity,
      double feederVelocity,
      double anglePosition,
      double intakeVelocity,
      double SideRoller) {
    topShooterController.setReference(TopVelocity, ControlType.kVelocity);
    bottomShooterController.setReference(BottomVelocity, ControlType.kVelocity);

    double ShooterAngle =
        anglePID.calculate(
            Units.rotationsToDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble()),
            anglePosition);
    angleMotor.setControl(
        new VoltageOut(MathUtil.clamp(MathUtil.applyDeadband(-ShooterAngle, 1), -12, 12)));

    FeedRoller.set(-feederVelocity);
    intake.set(-intakeVelocity);

    SideRollerController.setReference(SideRoller, ControlType.kVelocity);
  }
}
