// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** Add your docs here. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pidgy = new Pigeon2(1, null);
  private final StatusSignal<Double> yaw = pidgy.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yawVelocity = pidgy.getAngularVelocityZ();

  public GyroIOPigeon2(boolean phoenixDrive) {
    pidgy.getConfigurator().apply(new Pigeon2Configuration());
    pidgy.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(100);
    yawVelocity.setUpdateFrequency(100.0);
    pidgy.optimizeBusUtilization();
    if (phoenixDrive) {
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pidgy, pidgy.getYaw());
    } else {
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(() -> pidgy.getYaw().getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
