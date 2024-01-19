// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Constants;
import frc.robot.Subsytems.Flywheel.Flywheel;
import frc.robot.Subsytems.Flywheel.FlywheelFalcon;
import frc.robot.Subsytems.Flywheel.FlywheelIO;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakeIO;
import frc.robot.Subsytems.Intake.IntakeREV;

/** Add your docs here. */
public class RobotContainer {
    public static Flywheel flywheel;
    public static Intake intake;


    public RobotContainer() {

         switch(Constants.currentMode) {
        
            case REAL:
        
                flywheel = new Flywheel(new FlywheelFalcon());//real hardware file
                intake = new Intake(new IntakeREV());
        
            break;
        
            case SIM:
        
                flywheel = new Flywheel(new frc.robot.Subsytems.Flywheel.FlywheelSim() {});//sim mode
                intake = new Intake(new IntakeIO() {});
        
             break;
        
            default:
        
                flywheel = new Flywheel(new FlywheelIO() {});//replay mode
                intake = new Intake(new IntakeIO() {});
        
             break;
        
        }
        
    }
}
