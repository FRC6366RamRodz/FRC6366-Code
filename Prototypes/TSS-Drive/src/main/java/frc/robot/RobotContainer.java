// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/** Add your docs here. */

import frc.robot.SubSystems.SubRRDrive;
import frc.robot.Util.IO;
import frc.robot.Util.Constants.DT_Map;


public class RobotContainer {
    public static final SubRRDrive subRRDrive = new 
        SubRRDrive(DT_Map.leftFront, DT_Map.righFront, DT_Map.leftRear, DT_Map.rightRear, DT_Map.frontH, DT_Map.rearH, IO.getA(), DT_Map.stingerSolenoid, DT_Map.strafeSolenoid);
}
