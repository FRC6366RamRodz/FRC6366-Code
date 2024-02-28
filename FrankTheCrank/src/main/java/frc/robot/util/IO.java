// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
  public XboxController op = new XboxController(1);
  public XboxController dr = new XboxController(0);

  public IO() {}

  public boolean getOpX() {
    return op.getXButton();
  }

  public boolean getOpA() {
    return op.getAButton();
  }

  public boolean getOpY() {
    return op.getYButton();
  }

  public boolean getOPB() {
    return op.getBButton();
  }

  public boolean getOPLB() {
    return op.getLeftBumper();
  }

  public boolean getOpRB() {
    return op.getRightBumper();
  }

  public boolean getOpRTrigger() {
    if (op.getRightTriggerAxis() > 0.3) {
      return true;
    } else {
      return false;
    }
  }

  public void opRumble(double rumble) {
    op.setRumble(RumbleType.kBothRumble, rumble);
  }

  public void drRumble(double rumble) {
    dr.setRumble(RumbleType.kRightRumble, rumble);
  }
}
