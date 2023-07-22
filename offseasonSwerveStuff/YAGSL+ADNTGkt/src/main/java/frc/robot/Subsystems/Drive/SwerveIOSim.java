// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;


import swervelib.simulation.SwerveIMUSimulation;
import swervelib.simulation.SwerveModuleSimulation;

/** Add your docs here. */
public class SwerveIOSim {
    private SwerveIMUSimulation simulation = new SwerveIMUSimulation();
    private SwerveModuleSimulation mod1 = new SwerveModuleSimulation();

    

    public void setSimulation(SwerveIMUSimulation simulation) {
        this.simulation = simulation;
    }

    public void runSim(){
        
    }
}
