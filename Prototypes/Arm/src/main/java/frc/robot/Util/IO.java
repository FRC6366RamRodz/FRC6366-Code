// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class IO {
   private static final XboxController Driver = new XboxController(0);

   //manual percent out joysticks

   //arm I think
   public static double getLefftY() {
      return Driver.getLeftY();
   }

   //elbow I think
   public static double getRigthY() {
      return Driver.getRightY();
   }

   //Top position
   public static boolean getYbutton() {
      return Driver.getYButton();
   }

   //Middle Position
   public static boolean getXbutton() {
      return Driver.getXButton();
   }

   //Floor position
   public static boolean getBbutton() {
      return Driver.getBButton();
   }

   //Wrist Reset
   public static boolean getAbutton() {
      return Driver.getAButton();
   }

   public static boolean getLeftBumper() {
      return Driver.getLeftBumper();
   }

   public static boolean getRightBumper() {
      return Driver.getRightBumper();
   }

   public static double getPov(){
      return Driver.getPOV();
   }
}
