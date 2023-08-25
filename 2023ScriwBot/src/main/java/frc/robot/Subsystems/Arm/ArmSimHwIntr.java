// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmSimHwIntr implements ArmIO {
    private SingleJointedArmSim upperArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 1.0, 0.35, Units.inchesToMeters(25), 0, 90, true); 
    private SingleJointedArmSim lowerArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 1.0, 0.25, Units.inchesToMeters(10), 0, 75, true); 

    @Override
    public void updateInputs(ArmIoInputs inputs) {
        inputs.UpperMotorVelocity = upperArmSim.getVelocityRadPerSec();
        inputs.LowerMotorVelocity = lowerArmSim.getVelocityRadPerSec();
        inputs.LowerCoderPosition = lowerArmSim.getAngleRads();
        inputs.UpperCoderPosition = upperArmSim.getAngleRads();
    }

    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, double UsetPoint, double LsetPoint) {
    }
}
