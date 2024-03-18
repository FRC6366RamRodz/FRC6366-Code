// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface DriveIO {

  @AutoLog
  public static class DriveIOInputs { // where logged inputs are created
    public double leftPositionMeter = 0.0;
    public double rightPositionMeter = 0.0;

    public double leftVelocity = 0.0;
    public double rightVelocity = 0.0;

    public double leftAvgAmps =
        0.0; // technically could use a double array but uneeded complexity, both motors should see
    // similar amp draw
    public double rightAvgAmps = 0.0;

    public double leftAvgVolts =
        0.0; // technically could use a double array but uneeded complexity, both motors should see
    // similar volts
    public double rightAvgVolts = 0.0;

    public double leftAvgTemp = 0; // only available for NEO/REV motors
    public double rightAvgTemp = 0;

    public Rotation2d gyroYaw =
        new Rotation2d(); // needed for simulation, ignore if no simulation needed and no gyro
    // present
  }

  // globaly available classes

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}
}
