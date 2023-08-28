// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmSimHwIntr;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveIOSim;

/** Add your docs here. */
public class RobotContainer {
    public static final Arm arm = new Arm(new ArmSimHwIntr());
    public static final Drive drive = new Drive(new DriveIOSim());
}
