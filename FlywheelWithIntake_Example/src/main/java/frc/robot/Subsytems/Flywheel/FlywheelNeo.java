// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsytems.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class FlywheelNeo implements FlywheelIO {
    private final CANSparkMax FLYWHEEL_MOTOR = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder FLYWHEEL_ENCODER = FLYWHEEL_MOTOR.getEncoder();
    private final SparkPIDController FLYWHEEL_PIDCONTROLLER = FLYWHEEL_MOTOR.getPIDController();

    public FlywheelNeo() {
        FLYWHEEL_MOTOR.restoreFactoryDefaults(false);
        FLYWHEEL_MOTOR.setCANTimeout(250);
        FLYWHEEL_MOTOR.setInverted(false);

        int smartMotionSlot = 0;

        FLYWHEEL_PIDCONTROLLER.setP(0.00005, smartMotionSlot);
        FLYWHEEL_PIDCONTROLLER.setI(0, smartMotionSlot);
        FLYWHEEL_PIDCONTROLLER.setD(0, smartMotionSlot);
        FLYWHEEL_PIDCONTROLLER.setFF(0.000015, smartMotionSlot);
        FLYWHEEL_PIDCONTROLLER.setOutputRange(-1, 1, smartMotionSlot);
        FLYWHEEL_PIDCONTROLLER.setIZone(0, smartMotionSlot);
        
        FLYWHEEL_MOTOR.burnFlash();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs){

        inputs.VelocityRPM = FLYWHEEL_ENCODER.getVelocity();
        inputs.accelerationRPM = 0.0; //rev doesnt support for some reason

        inputs.Amps = FLYWHEEL_MOTOR.getOutputCurrent();
        inputs.Volts = FLYWHEEL_MOTOR.getAppliedOutput();

        inputs.Temp = FLYWHEEL_MOTOR.getMotorTemperature();
    }

     @Override
    public void setVelocity(double VelocityRPM){
        FLYWHEEL_PIDCONTROLLER.setReference(VelocityRPM, ControlType.kVelocity, 0);
    }
}
