// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;
import swervelib.simulation.SwerveIMUSimulation;
import swervelib.simulation.SwerveModuleSimulation;

/** Add your docs here. */
public class DriveIOSim implements SwerveIO {
    private SwerveIMUSimulation simulation = new SwerveIMUSimulation();
    private SwerveModuleSimulation LeftFront = new SwerveModuleSimulation();
    private SwerveModuleSimulation LeftRear = new SwerveModuleSimulation();
    private SwerveModuleSimulation RightFront = new SwerveModuleSimulation();
    private SwerveModuleSimulation RightRear = new SwerveModuleSimulation();

    public void updateInputs(SwerveIOInputs inputs) {
        
    }
}
