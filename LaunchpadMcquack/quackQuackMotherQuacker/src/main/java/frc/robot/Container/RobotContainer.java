// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import frc.robot.Subsystem.SubDriveTrain;
import frc.robot.Util.Constants.DT_Map;

/** Add your docs here. */
public class RobotContainer {
    public static final SubDriveTrain DiveTrain = new 
    SubDriveTrain(DT_Map.Lft_Frnt, DT_Map.Rht_Frnt, DT_Map.Lft_Rr, DT_Map.Rht_Rr);
}
