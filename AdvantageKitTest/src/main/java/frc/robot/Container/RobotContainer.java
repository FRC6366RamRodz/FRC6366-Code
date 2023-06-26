// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import com.ctre.phoenix.signals.IOutputSignal;

import frc.robot.Robot;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveHWIntr;
import frc.robot.Subsystems.Drive.DriveIo;
import frc.robot.Subsystems.Drive.DriveIo.DriveIOInputs;
import frc.robot.Util.IO;

/** Add your docs here. */
public class RobotContainer {
    public static final Drive Drive = new 
    Drive(new DriveIo(){});
}
