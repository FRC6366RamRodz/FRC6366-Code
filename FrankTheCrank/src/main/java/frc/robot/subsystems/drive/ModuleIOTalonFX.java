// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = 4;
  private final double TURN_GEAR_RATIO = 13.3714;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0: // fl
        driveTalon = new TalonFX(1, "Swerve");
        turnTalon = new TalonFX(11, "Swerve");
        cancoder = new CANcoder(1, "Swerve");
        absoluteEncoderOffset = new Rotation2d(1.824+ Math.PI); // MUST BE CALIBRATED
        break;
      case 1: // fr
        driveTalon = new TalonFX(2, "Swerve");
        turnTalon = new TalonFX(12, "Swerve");
        cancoder = new CANcoder(2, "Swerve");
        absoluteEncoderOffset = new Rotation2d(0.821 + Math.PI); // MUST BE CALIBRATED
        break;
      case 2: // bl
        driveTalon = new TalonFX(3, "Swerve");
        turnTalon = new TalonFX(13, "Swerve");
        cancoder = new CANcoder(3, "Swerve");
        absoluteEncoderOffset = new Rotation2d(-0.779 + Math.PI); // MUST BE CALIBRATED
        break;
      case 3: // br
        driveTalon = new TalonFX(4, "Swerve");
        turnTalon = new TalonFX(14, "Swerve");
        cancoder = new CANcoder(4, "Swerve");
        absoluteEncoderOffset = new Rotation2d(1.585 + Math.PI); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Voltage.PeakForwardVoltage = 12.0;
    turnConfig.Voltage.PeakReverseVoltage = -12.0;
    // TUNE PID CONSTANTS
    turnConfig.Slot0.kP = 8.0;
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = 0.0;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -30;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    setTurnBrakeMode(true);

    turnTalon.getConfigurator().apply(turnConfig);

    var turnEncoder = new CANcoderConfiguration();
    turnEncoder.MagnetSensor.MagnetOffset = -absoluteEncoderOffset.getRotations();

    cancoder.getConfigurator().apply(turnEncoder);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition, turnPosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(new Rotation2d(0));
    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnPosition(double moduleAngle) {
    
    turnTalon.setControl(new PositionVoltage(moduleAngle).withSlot(0).withOverrideBrakeDurNeutral(true));
  }
  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
