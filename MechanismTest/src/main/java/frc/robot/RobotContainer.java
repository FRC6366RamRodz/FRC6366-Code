// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.armTalonFx;

/** Add your docs here. */
public class RobotContainer {
  public static Arm arm = new Arm(new armTalonFx());
}
