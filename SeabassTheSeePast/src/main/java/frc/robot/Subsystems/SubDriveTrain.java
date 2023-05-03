// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.DT_STG;

/** Add your docs here. */
public class SubDriveTrain {//initialization variables should be descriptive all CAPS and underscores seperating words
    private static TalonSRX LEFT_FRONT_MOTOR;
    private static TalonSRX RIGHT_FRONT_MOTOR;
    private static VictorSPX LEFT_REAR_MOTOR;
    private static VictorSPX RIGHT_REAR_MOTOR;
    private static TalonSRX FRONT_DROP_MOTOR;
    private static TalonSRX REAR_DROP_MOTOR;
    private static Solenoid FRONT_DROP_SOLENOID;
    private static Solenoid REAR_DROP_SOLENOID;

//call variables should be camel case with descriptive names.
    public SubDriveTrain(int leftFrontMotor, int rightFrontMotor, int leftRearMotor, int rightRearMotor, int frontDropMotor, int rearDropMotor, int frontDropSolenoid, int rearDropSolenoid) {
        // these list can become tiring so put a copy paste camment of all motors you are working with
        /*LEFT_FRONT_MOTOR
        LEFT_REAR_MOTOR
        RIGHT_FRONT_MOTOR
        RIGHT_REAR_MOTOR
        FRONT_DROP_MOTOR
        REAR_DROP_MOTOR
         */
        LEFT_FRONT_MOTOR = new TalonSRX(leftFrontMotor);
        LEFT_REAR_MOTOR = new VictorSPX(leftRearMotor);
        RIGHT_FRONT_MOTOR = new TalonSRX(rightFrontMotor);
        RIGHT_REAR_MOTOR = new VictorSPX(rightRearMotor);
        FRONT_DROP_MOTOR = new TalonSRX(frontDropMotor);
        REAR_DROP_MOTOR = new TalonSRX(rearDropMotor);

        FRONT_DROP_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, frontDropSolenoid);
        REAR_DROP_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, rearDropSolenoid);

        LEFT_REAR_MOTOR.follow(LEFT_FRONT_MOTOR);
        RIGHT_REAR_MOTOR.follow(RIGHT_FRONT_MOTOR);

        LEFT_FRONT_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Tnk_Acc_Rte);
        RIGHT_FRONT_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Tnk_Acc_Rte);

        LEFT_FRONT_MOTOR.setInverted(false);
        LEFT_REAR_MOTOR.setInverted(false);
        RIGHT_FRONT_MOTOR.setInverted(false);
        RIGHT_FRONT_MOTOR.setInverted(false);
        FRONT_DROP_MOTOR.setInverted(false);
        REAR_DROP_MOTOR.setInverted(false);

        LEFT_FRONT_MOTOR.setNeutralMode(NeutralMode.Brake);
        LEFT_REAR_MOTOR.setNeutralMode(NeutralMode.Brake);
        RIGHT_FRONT_MOTOR.setNeutralMode(NeutralMode.Brake);
        RIGHT_REAR_MOTOR.setNeutralMode(NeutralMode.Brake);
        FRONT_DROP_MOTOR.setNeutralMode(NeutralMode.Brake);
        REAR_DROP_MOTOR.setNeutralMode(NeutralMode.Brake);

        LEFT_FRONT_MOTOR.configPeakCurrentLimit(Constants.DT_STG.Pk_Crrnt_Lmt);
        RIGHT_FRONT_MOTOR.configPeakCurrentLimit(Constants.DT_STG.Pk_Crrnt_Lmt);
        FRONT_DROP_MOTOR.configPeakCurrentLimit(Constants.DT_STG.Pk_Crrnt_Lmt);
        REAR_DROP_MOTOR.configPeakCurrentLimit(Constants.DT_STG.Pk_Crrnt_Lmt);

        LEFT_FRONT_MOTOR.configPeakCurrentDuration(DT_STG.Pk_Crrnt_Lmt_Tm);
        RIGHT_FRONT_MOTOR.configPeakCurrentDuration(DT_STG.Pk_Crrnt_Lmt_Tm);
        FRONT_DROP_MOTOR.configPeakCurrentDuration(DT_STG.Pk_Crrnt_Lmt_Tm);
        REAR_DROP_MOTOR.configPeakCurrentDuration(DT_STG.Pk_Crrnt_Lmt_Tm);
    }

    public void runDriveTrain (double leftY, double rightX, double leftMotor, double rightMotor, double frontDropMotor, double rearDropMotor, boolean frontDrop, boolean rearDrop, boolean limelightButton, boolean slowMode) {
        double left, right, dropfm/*drop front motor */, droprm/*drop rear motor */, targetX, kt, mincommandt, headingerrort, steeringadjustt, ks, mincommands, headingerrors, steeringadjusts, pmt, pms; //low level local varriables should be all lowercase and shorter (may be abreviations, explain in a comment).

        targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        kt = 0.0045;
        ks = 0.0045;

        mincommandt = 0.01;
        mincommands = 0.08;
        
        headingerrort = targetX;
        headingerrors = targetX;

        steeringadjustt = 0.0;
        steeringadjusts = 0.0;

        pmt = headingerrort / Math.abs(headingerrort);
        pms = headingerrors / Math.abs(headingerrors);

        //automation loops
        if (limelightButton && frontDrop == false && Math.abs(headingerrort) > 0.5) {
            steeringadjustt = kt*headingerrort*pmt+mincommandt;

            left = steeringadjustt+(leftY*0.5);
            right = steeringadjustt+(leftY*0.5);
            dropfm = 0;
            droprm = 0;

        } else if (limelightButton && frontDrop && Math.abs(headingerrors) > 0.3) {
            steeringadjusts = ks*headingerrors*pms+mincommands;

            dropfm = steeringadjusts+(rightX*0.5);
            droprm = steeringadjusts+(rightX*0.5);
            left = 0;
            right = 0;

        } else if (slowMode && rearDrop == false) {

            left = leftMotor*Constants.DT_STG.Slw_Md_Tnk_Spd;
            right = rightMotor*Constants.DT_STG.Slw_Md_Tnk_Spd;
            dropfm = frontDropMotor*Constants.DT_STG.Slw_Md_Tnk_Spd;
            droprm = rearDropMotor*Constants.DT_STG.Slw_Md_Tnk_Spd;

        } else if (slowMode && rearDrop && frontDrop) {
            left = leftMotor*Constants.DT_STG.Slw_Md_Str_Spd;
            right = rightMotor*Constants.DT_STG.Slw_Md_Str_Spd;
            dropfm = frontDropMotor*Constants.DT_STG.Slw_Md_Str_Spd;
            droprm = rearDropMotor*Constants.DT_STG.Slw_Md_Str_Spd;
        } else {
            left = leftMotor;
            right = rightMotor;
            dropfm = frontDropMotor;
            droprm = rearDropMotor;
        }

        //acceleration rates
        if (rearDrop && frontDrop == false) {
            FRONT_DROP_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Tnk_Acc_Rte);
            REAR_DROP_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Tnk_Acc_Rte);
        } else {
            FRONT_DROP_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Str_Acc_Rte);
            REAR_DROP_MOTOR.configOpenloopRamp(Constants.DT_STG.Dt_Str_Acc_Rte);
        }

        LEFT_FRONT_MOTOR.set(ControlMode.PercentOutput, left);
        RIGHT_FRONT_MOTOR.set(ControlMode.PercentOutput, right);
        FRONT_DROP_MOTOR.set(ControlMode.PercentOutput, dropfm);
        REAR_DROP_MOTOR.set(ControlMode.PercentOutput, droprm);

        FRONT_DROP_SOLENOID.set(frontDrop);
        REAR_DROP_SOLENOID.set(rearDrop);
        

    }
}


