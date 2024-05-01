// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Controller input for timed based systems. */
public class IO {
  public XboxController op = new XboxController(1);//operator controller should be port 1
  public XboxController dr = new XboxController(0);//driver controller should be port 0

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

    public boolean getOpLTrigger() {
    if (op.getLeftTriggerAxis() > 0.3) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getDrRTrigger() {
    if (dr.getRightTriggerAxis() > 0.3) {
      return true;
    } else {
      return false;
    }
  }

  public double getOpRightY() {
    return op.getRightY();
  }

  public boolean getOPLYDown() {
    if (op.getLeftY() > 0.5) {
      return true;
    } else {
      return false;
    }
  }

    public boolean getOPLYUp() {
    if (op.getLeftY() < -0.5) {
      return true;
    } else {
      return false;
    }
  }

  public double getDpad() {
    return op.getPOV();
  }

  public boolean DriveLTPressed() {
    return dr.getLeftTriggerAxis() > 0.8;
  }

  public void opRumble(double rumble) {
    op.setRumble(RumbleType.kBothRumble, rumble);
  }

  public void drRumble(double rumble) {
    dr.setRumble(RumbleType.kBothRumble, rumble);
  }

  public void drLightRumble(double rumble) {
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

  public boolean getDrAbutton() {
    return dr.getAButton();
  }

  public boolean getDrLeftBumper() {
    return dr.getLeftBumper();
  }
}
