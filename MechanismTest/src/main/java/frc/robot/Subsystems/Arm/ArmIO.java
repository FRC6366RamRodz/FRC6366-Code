// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double EncoderAbsolutePosition = 0.0;
    public double MotorPosition = 0.0;
    public double MechanismLocation = 0.0;
    public double MotorVelocity = 0.0;
    public double EncoderVelocity = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setMotors(Rotation2d SetPoint) {}
}
