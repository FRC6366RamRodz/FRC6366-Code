// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double TopVelocity = 0.0;
    public double BottomVelocity = 0.0;
    public double feederVelocity = 0.0;
    public double angleVelocity = 0.0;
    public double anglePosition = 0.0;
    public double intakeVelocity = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setMotors(
      double TopVelocity,
      double BottomVelocity,
      double feederVelocity,
      double anglePosition,
      double intakeVelocity) {}
}
