// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class SubDriveTrain {//initialization variables should be descriptive all CAPS and underscores seperating words
    private static TalonFX LEFT_FRONT_MOTOR;
    private static TalonFX RIGHT_FRONT_MOTOR;
    private static TalonFX LEFT_REAR_MOTOR;
    private static TalonFX RIGHT_REAR_MOTOR;

    //call variables should be camel case with descriptive names.
    public SubDriveTrain(int leftFrontMotorId, int rightFrontMotorId, int leftRearMotorId, int rightRearMotorId) {
         // these list can become tiring so put a copy paste camment of all motors you are working with
      /*LEFT_FRONT_MOTOR
        RIGHT_FRONT_MOTOR
        LEFT_REAR_MOTOR
        RIGHT_REAR_MOTOR
         */

         //when they start to get longer "white space" will be you best friend

        LEFT_FRONT_MOTOR = new TalonFX(leftFrontMotorId); // keep these lists in the same order left right left right, left left right right etc...
        RIGHT_FRONT_MOTOR = new TalonFX(rightFrontMotorId);
        LEFT_REAR_MOTOR = new TalonFX(leftRearMotorId);
        RIGHT_REAR_MOTOR = new TalonFX(rightRearMotorId);

        LEFT_REAR_MOTOR.follow(LEFT_FRONT_MOTOR); // we follow/slave drive motors together to decrease work and failure cases.
        RIGHT_REAR_MOTOR.follow(RIGHT_FRONT_MOTOR);

        LEFT_FRONT_MOTOR.setInverted(false); //inverts allow us to fix direction of power issues without changing any controll loop code.
        RIGHT_FRONT_MOTOR.setInverted(false);
        LEFT_REAR_MOTOR.setInverted(false);
        RIGHT_REAR_MOTOR.setInverted(false);

        LEFT_FRONT_MOTOR.setNeutralMode(NeutralMode.Brake);//causes the motor to slow itself down generally used in open loop controll. Normally set to coast for closed loop/PID controll
        RIGHT_FRONT_MOTOR.setNeutralMode(NeutralMode.Brake);
        LEFT_REAR_MOTOR.setNeutralMode(NeutralMode.Brake);
        RIGHT_REAR_MOTOR.setNeutralMode(NeutralMode.Brake);

    }

    public void runDriveTrain(double leftMotor, double rightMotor) { //call variables should be camel case with descriptive names
        //this is your output loop secondary filtering is the only thing that should be in here beyond you motor setters.
        LEFT_FRONT_MOTOR.set(ControlMode.PercentOutput, leftMotor);
        RIGHT_FRONT_MOTOR.set(ControlMode.PercentOutput, rightMotor);
    }
}
