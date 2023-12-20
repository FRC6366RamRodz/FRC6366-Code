// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
  private static final XboxController driver = new XboxController(0);

  public static double getLeftY() {
    return MathUtil.applyDeadband(-driver.getLeftY() * Math.abs(driver.getLeftY()), 0.15);
  }

  public static double getRightX() {
    return MathUtil.applyDeadband(-driver.getRightX() * Math.abs(driver.getRightX()), 0.15);
  }
}
