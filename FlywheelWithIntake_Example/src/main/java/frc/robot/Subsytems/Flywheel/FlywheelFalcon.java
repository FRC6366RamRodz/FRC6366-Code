// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class FlywheelFalcon implements FlywheelIO {
  private final TalonFX FLYWHEEL_MOTOR = new TalonFX(5, null);

  private final StatusSignal<Double> velocityRPS = FLYWHEEL_MOTOR.getVelocity();
  private final StatusSignal<Double> accelerationRPS = FLYWHEEL_MOTOR.getAcceleration();

  public FlywheelFalcon() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
    slot0Configs.kD = 0.01; // An acceleration of 1 rps/s results in 0.01 V output

    FLYWHEEL_MOTOR.getConfigurator().apply(config);
    FLYWHEEL_MOTOR.getConfigurator().apply(slot0Configs);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocityRPS, accelerationRPS);

    FLYWHEEL_MOTOR.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityRPS, accelerationRPS);

    inputs.VelocityRPM = velocityRPS.getValueAsDouble() * 60.0000024;
    inputs.accelerationRPM = accelerationRPS.getValueAsDouble() * 60.0000024;

    inputs.Amps = FLYWHEEL_MOTOR.getStatorCurrent().getValueAsDouble();
    inputs.Volts = FLYWHEEL_MOTOR.getMotorVoltage().getValueAsDouble();

    inputs.Temp = 0.0; // falcons cant read Tempeture
  }

  @Override
  public void setVelocity(double VelocityRPM) {
    FLYWHEEL_MOTOR.setControl(
        new VelocityVoltage(0)
            .withSlot(0)
            .withVelocity(VelocityRPM * 0.01666666666)
            .withFeedForward(0.5));
  }
}
