// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmSimHwIntr implements ArmIO {
    private SingleJointedArmSim upperArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 10.0, 0.08, Units.inchesToMeters(38), Units.degreesToRadians(-90), Units.degreesToRadians(90), false); 
    private SingleJointedArmSim lowerArmSim = new SingleJointedArmSim(DCMotor.getNEO(1), 5.0, 0.004, Units.inchesToMeters(15), Units.degreesToRadians(-35), Units.degreesToRadians(90), false); 


    @Override
    public void updateInputs(ArmIoInputs inputs) {

        upperArmSim.update(0.02);
        lowerArmSim.update(0.02);

        inputs.UpperMotorVelocity = Units.radiansPerSecondToRotationsPerMinute(upperArmSim.getVelocityRadPerSec());
        inputs.LowerMotorVelocity = Units.radiansPerSecondToRotationsPerMinute(lowerArmSim.getVelocityRadPerSec());
        inputs.LowerCoderPosition = Units.radiansToDegrees(lowerArmSim.getAngleRads());
        inputs.UpperCoderPosition = Units.radiansToDegrees(upperArmSim.getAngleRads());
    }

    @Override
    public void setSpeed(double upperSpeed, double lowerSpeed, double UsetPoint, double LsetPoint){
        upperArmSim.setInput(MathUtil.clamp(upperSpeed*12, -12.0, 12.0));
        lowerArmSim.setInput(MathUtil.clamp(lowerSpeed*12, -12.0, 12.0));
    }
}
