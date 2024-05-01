// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs{
    public double UpperArmPosition = 0.0;
    public double LowerArmPosition = 0.0;
    public double UpperTemp = 0.0;
    public double LowerTemp = 0.0;
    public double UpperPIDError = 0.0;
    public double LowerPIDError = 0.0;
    public double IntakeLeftSpeed = 0.0;
    public double IntakeRightSpeed = 0.0;
    public boolean UpperBrake = true;
    public boolean LowerBrake = true;
    public boolean IntakeWrist = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(double UpperAngle, double LowerArm, boolean WristSet, double Intake) {}
}
