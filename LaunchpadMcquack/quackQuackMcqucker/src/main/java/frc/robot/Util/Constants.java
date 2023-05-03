// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

/** Add your docs here. */
public class Constants {

    public static class DT_Map/*Drive Train motor IDs*/{//global varriables shoule be cammel case and abbreviations with underscores seperatring words
        public static final int Lft_Frnt = 0;//left front
        public static final int Rht_Frnt = 1;//right front
        public static final int Lft_Rr = 2;//left rear
        public static final int Rht_Rr = 3;//right rear
    }

    public static class DT_STG/*Drive Train Settings */ {
        public static final double DeadBand = 0.05; //sometimes an abbreviation is more problamatic in those cases leave them camel case
        public static final double Drv_Sens = 0.55;//driver sensitivity
    }
}
