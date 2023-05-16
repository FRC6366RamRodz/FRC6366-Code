// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.IO;
import frc.robot.Util.Constants.AR_SET;

/** Add your docs here. */
public class SubArm {
    private SparkMaxPIDController ARM_PIDCONTROLLER;
    private SparkMaxPIDController ELBOW_PIDCONTROLLER;
    //create motors
    private CANSparkMax ARM_MOTOR;
    private CANSparkMax ELBOW_MOTOR;
    //Create Limit Switches
    private SparkMaxLimitSwitch ARM_FORWARD_LIMIT;
    private SparkMaxLimitSwitch ARM_REVERSE_LIMIT;
    private SparkMaxLimitSwitch ELBOW_FORWARD_LIMIT;
    private SparkMaxLimitSwitch ELBOW_REVERSE_LIMIT;
    //create SolenoidsPP
    private final Solenoid ARM_BRAKE;
    private final Solenoid ELBOW_BRAKE;
    //create encoders
    private RelativeEncoder ARM_ENCDOER;
    private RelativeEncoder ELBOW_ENCODER;
    //create Solenoids
    private Solenoid ArmBrakeExternal;


    public SubArm (int elbow, int arm, int armBrake, int elbowBrake) {
        //initialize solenoids
        ArmBrakeExternal = new Solenoid(PneumaticsModuleType.REVPH, 7);
        //initialize Motors
        ARM_MOTOR = new CANSparkMax(arm, MotorType.kBrushless);
        ELBOW_MOTOR = new CANSparkMax(elbow, MotorType.kBrushless);
        
        // reset Motors
        ARM_MOTOR.restoreFactoryDefaults();
        ELBOW_MOTOR.restoreFactoryDefaults();
       
        // set Encoders
        ARM_ENCDOER = ARM_MOTOR.getEncoder();
        ELBOW_ENCODER = ELBOW_MOTOR.getEncoder();
        // set PID controller
        ARM_PIDCONTROLLER = ARM_MOTOR.getPIDController();
        ELBOW_PIDCONTROLLER = ELBOW_MOTOR.getPIDController();
        // set Pnuematics
        ARM_BRAKE = new Solenoid(PneumaticsModuleType.REVPH, armBrake);
        ELBOW_BRAKE = new Solenoid(PneumaticsModuleType.REVPH, elbowBrake);
      
        // set limit switch
        ARM_FORWARD_LIMIT = ARM_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ARM_REVERSE_LIMIT = ARM_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ELBOW_FORWARD_LIMIT = ELBOW_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ELBOW_REVERSE_LIMIT = ELBOW_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        // enable limit switches
        ARM_FORWARD_LIMIT.enableLimitSwitch(true);
        ARM_REVERSE_LIMIT.enableLimitSwitch(true);
        ELBOW_FORWARD_LIMIT.enableLimitSwitch(true);
        ELBOW_REVERSE_LIMIT.enableLimitSwitch(true);
        /*
        ARM_PIDCONTROLLER
        ELBOW_PIDCONTROLLER 
         */
        int smartMotionSlot = 0;
        //set P
        ARM_PIDCONTROLLER.setP(AR_SET.kP);
        ELBOW_PIDCONTROLLER.setP(AR_SET.kP);
        //set I
        ARM_PIDCONTROLLER.setI(AR_SET.kI);
        ELBOW_PIDCONTROLLER.setI(AR_SET.kI);
        //set D
        ARM_PIDCONTROLLER.setD(AR_SET.kD);
        ELBOW_PIDCONTROLLER.setD(AR_SET.kD);
        //set FF
        ARM_PIDCONTROLLER.setFF(AR_SET.kFF);
        ELBOW_PIDCONTROLLER.setFF(AR_SET.kFF);
        //set Output range
        ARM_PIDCONTROLLER.setOutputRange(AR_SET.kMinOutput, AR_SET.kMaxOutput);
        ELBOW_PIDCONTROLLER.setOutputRange(AR_SET.kMinOutput, AR_SET.kMaxOutput);
        //set Smart max Vel
        ARM_PIDCONTROLLER.setSmartMotionMaxVelocity(AR_SET.maxVel, smartMotionSlot);
        ELBOW_PIDCONTROLLER.setSmartMotionMaxVelocity(AR_SET.maxVel, smartMotionSlot);
        //set Smart min Vel
        ARM_PIDCONTROLLER.setSmartMotionMinOutputVelocity(AR_SET.minVel, smartMotionSlot);
        ELBOW_PIDCONTROLLER.setSmartMotionMinOutputVelocity(AR_SET.minVel, smartMotionSlot);
        //set Smart max Acc
        ARM_PIDCONTROLLER.setSmartMotionMaxAccel(AR_SET.maxAcc, smartMotionSlot);
        ELBOW_PIDCONTROLLER.setSmartMotionMaxAccel(AR_SET.maxAcc, smartMotionSlot);
        //set Allwd err
        ARM_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(AR_SET.allowedErr, smartMotionSlot);
        ELBOW_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(AR_SET.allowedErr, smartMotionSlot);
        //smart dashboard values
        SmartDashboard.putNumber("Set Arm Position", 0);
        SmartDashboard.putNumber("Set Wrist Position", 0);
        SmartDashboard.putBoolean("Manual Position", false);
        
        ARM_MOTOR.burnFlash();
        ELBOW_MOTOR.burnFlash();
    }

    public void runArmSmartMotion(double setPointA, double setPointE, boolean home){
        //setMotorPositions
        double setElbow;
        boolean manual_mode;
        manual_mode = SmartDashboard.getBoolean("Manual Position", false);
       
       
        // manual mode allows the positions to be set from the dash while the other mode uses set positions
        if(manual_mode == true) {
        //dash values for positions
        setPointA = SmartDashboard.getNumber("Set Arm Position", 0);
        setPointE = SmartDashboard.getNumber("Set Wrist Position", 0);
            if (ARM_ENCDOER.getPosition() >25) {
                setElbow = setPointE-9;
            } else if (ARM_ENCDOER.getPosition() >13) {
                setElbow = setPointE-4;
            } else if (ARM_ENCDOER.getPosition() > 9) {
                setElbow = setPointE-1;
            } else {
            setElbow = setPointE;
            }
        
        
        //sets motors to smart motion and gives a position
        ARM_PIDCONTROLLER.setReference(setPointA, CANSparkMax.ControlType.kSmartMotion);
        ELBOW_PIDCONTROLLER.setReference(setElbow, CANSparkMax.ControlType.kSmartMotion);
        } else {
            if (ARM_ENCDOER.getPosition() >20) {
                setElbow = setPointE-7.3;
            } else if (ARM_ENCDOER.getPosition() >13) {
                setElbow = setPointE-4;
            } else if (ARM_ENCDOER.getPosition() > 9) {
                setElbow = setPointE-1;
            } else {
            setElbow = setPointE;
            }
                //hard coded position, adjustable in Robot.teleopPeriodic
                ARM_PIDCONTROLLER.setReference(setPointA, CANSparkMax.ControlType.kSmartMotion);
                ELBOW_PIDCONTROLLER.setReference(setElbow, CANSparkMax.ControlType.kSmartMotion);
        }

        //set Brakes
        if (ARM_ENCDOER.getPosition() >= setPointA-0.15 && ARM_ENCDOER.getPosition() <= setPointA+0.15) {
            ARM_BRAKE.set(false);
        } else {
            ARM_BRAKE.set(true);
        }
        if (ELBOW_ENCODER.getPosition() >= setElbow-0.15 && ELBOW_ENCODER.getPosition() <= setElbow+0.15) {
            ELBOW_BRAKE.set(false);
        } else {
            ELBOW_BRAKE.set(true);
        }
        if (ARM_REVERSE_LIMIT.isPressed() == true) {
            ARM_ENCDOER.setPosition(0);
    } else {
            
    }

    if (IO.getPovOp() == 180) {
        ArmBrakeExternal.set(true);
    } else {
        ArmBrakeExternal.set(false);
    }
    if (home == true) {
        ELBOW_REVERSE_LIMIT.enableLimitSwitch(true);
    } else {
        ELBOW_REVERSE_LIMIT.enableLimitSwitch(false);
    }
    if (home==true && ELBOW_REVERSE_LIMIT.isPressed()==true) {
        ELBOW_ENCODER.setPosition(0);
    } else {}
    }

    public double getArmEncoder() {
        return ARM_ENCDOER.getPosition();
    }

    public double getElbowEnocoder() {
        return ELBOW_ENCODER.getPosition();
    }
//get manual mode
    public boolean getManualMode() {
        return SmartDashboard.getBoolean("ManualMode", false);
    }

    public double getArmCurrent() {
        return ARM_MOTOR.getOutputCurrent();
    }

    public double getElbowCurrent() {
        return ELBOW_MOTOR.getOutputCurrent();
    }

}
