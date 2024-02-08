// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class ControllConstants {

  public static final double DeadBand = 0.14;
  public static final double DeadBandTriger = 0.05;

  public static class ARM {
    public static final double kP = 0.005;
    public static final double kI = 0;
    public static final double kD = 0.00000001;
    public static final double kIz = 0;
    public static final double kFF = 0.000015;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 20000;
    public static final double maxVel = 20000;
    public static final double maxAcc = 20000;
    public static final double minVel = 0;
    public static final double allowedErr = 0.01;
    public static final double ArmUOffset = 63.369;
    public static final double ArmLOffset = 303.75;
  }
}
