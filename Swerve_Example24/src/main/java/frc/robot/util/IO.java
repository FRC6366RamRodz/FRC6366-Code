// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
  public XboxController dr = new XboxController(0);

  public IO() {}

  public void drRumble(double rumble) {
    dr.setRumble(RumbleType.kRightRumble, rumble);
  }

  public boolean getDrY() {
   return dr.getYButton();
  }

  public boolean getDrRb() {
    if (dr.getRightBumperPressed() || dr.getRightBumperReleased()) {
      return true;
    } else {
      return false;
    }
  }
}
