// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveFalcon implements DriveIO {
  private static final double gearRatio =
      13.0; // roughly 10ft a second with six inch wheels and dual neo per side

  private final TalonFX LEFT_FRONT =
      new TalonFX(0, null); // identify device type, set a call name, offer an ID # for that motor.
  private final TalonFX RIGHT_FRONT = new TalonFX(1, null);
  private final TalonFX LEFT_REAR = new TalonFX(2, null);
  private final TalonFX RIGHT_REAR = new TalonFX(3, null);

  private final StatusSignal<Double> leftPosition = LEFT_FRONT.getPosition();
  private final StatusSignal<Double> rightPosition = RIGHT_FRONT.getPosition();

  private final StatusSignal<Double> leftVelocity = LEFT_FRONT.getVelocity();
  private final StatusSignal<Double> rightVelocity = RIGHT_FRONT.getVelocity();

  // Optional pigeon gyro
  // private final Pigeon2 pigeon = new Pigeon2(20);

  public DriveFalcon() {
    var config = new TalonFXConfiguration(); // create a config for falcons, can be done all at once
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    LEFT_FRONT.getConfigurator().apply(config);
    RIGHT_FRONT.getConfigurator().apply(config);
    LEFT_REAR.getConfigurator().apply(config);
    RIGHT_REAR.getConfigurator().apply(config);
    LEFT_REAR.setControl(new Follower(LEFT_FRONT.getDeviceID(), false));
    RIGHT_REAR.setControl(new Follower(RIGHT_FRONT.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leftPosition, rightPosition); // Required for odometry, use faster rate

    LEFT_FRONT.optimizeBusUtilization();
    RIGHT_FRONT.optimizeBusUtilization();
    LEFT_REAR.optimizeBusUtilization();
    RIGHT_REAR.optimizeBusUtilization();

    /*optional pigeon stuff
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        pigeon.optimizeBusUtilization();
    */
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) { // logged values

    BaseStatusSignal.refreshAll(leftPosition, leftVelocity, rightPosition, rightVelocity);

    inputs.leftPositionRad = Units.rotationsToDegrees(leftPosition.getValueAsDouble() / gearRatio);
    inputs.rightPositionRad =
        Units.rotationsToRadians(rightPosition.getValueAsDouble() / gearRatio);

    inputs.leftAvgVolts =
        (LEFT_FRONT.getMotorVoltage().getValueAsDouble()
                + LEFT_REAR.getMotorVoltage().getValueAsDouble())
            / 2;
    inputs.rightAvgVolts =
        (RIGHT_FRONT.getMotorVoltage().getValueAsDouble()
                + RIGHT_REAR.getMotorVoltage().getValueAsDouble())
            / 2;

    inputs.leftVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftVelocity.getValueAsDouble()
                / gearRatio); // divide by gear ratio to get wheel speed or dont to get motor speed.
    inputs.rightVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(rightVelocity.getValueAsDouble() / gearRatio);

    inputs.leftAvgAmps =
        (LEFT_FRONT.getStatorCurrent().getValueAsDouble()
                + LEFT_REAR.getStatorCurrent().getValueAsDouble())
            / 2;
    inputs.rightAvgAmps =
        (RIGHT_FRONT.getStatorCurrent().getValueAsDouble()
                + RIGHT_REAR.getStatorCurrent().getValueAsDouble())
            / 2;

    inputs.leftAvgTemp = (0); // falcons do not have tempeture readings
    inputs.rightAvgTemp = (0);

    /*optional pigeon gyro
     * inputs.gyroYaw = Rotation2d.fromDegrees(pigeon.getYaw);
     */
    // odometry workaround may need the signs swapped
    inputs.gyroYaw = new Rotation2d(inputs.leftPositionRad - inputs.rightPositionRad);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) { // set values
    LEFT_FRONT.setControl(new VoltageOut(leftVolts * 12));
    RIGHT_FRONT.setControl(new VoltageOut(rightVolts * 12));
  }
}
