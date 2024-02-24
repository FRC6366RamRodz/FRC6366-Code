// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class armTalonFx implements ArmIO {
  public static TalonFX F_Arm = new TalonFX(1);

  public static CANcoder encoder = new CANcoder(1);

  public double gearRatio = 56/12;

  public armTalonFx() {
    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.CurrentLimits.StatorCurrentLimit = 35;
    armConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    armConfig.Feedback.SensorToMechanismRatio = gearRatio;

    armConfig.Slot0.kP = 30;
    armConfig.Slot0.kI = 0;
    armConfig.Slot0.kD = 0;
    armConfig.Slot0.kV = 0.12;
    armConfig.Slot0.kS = 0.1;
    armConfig.MotionMagic.MotionMagicAcceleration = 1000;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    armConfig.MotionMagic.MotionMagicJerk = 0;
    armConfig.ClosedLoopGeneral.ContinuousWrap = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(90);
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    F_Arm.getConfigurator().apply(new TalonFXConfiguration());
    F_Arm.getConfigurator().apply(armConfig);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = -0.162842;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.EncoderAbsolutePosition = encoder.getAbsolutePosition().getValueAsDouble() * 360;
    inputs.EncoderAbsolutePosition = encoder.getVelocity().getValueAsDouble() * 60;
    inputs.MotorPosition = F_Arm.getPosition().getValueAsDouble() * 360;
    inputs.MotorVelocity = F_Arm.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void setMotors(Rotation2d SetPoint) {
    F_Arm.setControl(new MotionMagicVoltage(SetPoint.getRotations()).withSlot(0));

    if (F_Arm.getPosition().getValueAsDouble() > new Rotation2d(Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble())).minus(new Rotation2d(Units.degreesToRadians(0.5))).getRotations() && F_Arm.getPosition().getValueAsDouble() < new Rotation2d(Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble())).plus(new Rotation2d(Units.degreesToRadians(0.5))).getRotations()) {

    } else {
      F_Arm.setPosition(new Rotation2d(Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble())).getRotations());
    }
  }
}
