// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double VelocityRPM = 0.0;
    public double accelerationRPM = 0.0;
    public double Volts = 0.0;
    public double Amps = 0.0;
    public double Temp = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVelocity(double VelocityRPM) {}
}
