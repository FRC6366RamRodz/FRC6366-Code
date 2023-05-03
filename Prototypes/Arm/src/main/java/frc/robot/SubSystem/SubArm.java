// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.IO;

/** Add your docs here. */
public class SubArm {
    //Create motor(S)
    private SparkMaxPIDController ARM_PIDCONTROLLER;
    private SparkMaxPIDController WRIST_PIDCONTROLLER;
    private RelativeEncoder ARM_ENCODER;
    private RelativeEncoder WRIST_ENCODER;
    private CANSparkMax WRIST_MOTOR;
    private CANSparkMax ARM_MOTOR;
    private CANSparkMax Left_CLAW_MOTOR;
    private CANSparkMax RIGHT_CLAW_MOTOR;
    public double ArmPercentOut, WristPercentOut, wristOut;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private SparkMaxLimitSwitch ARM_forwardLimit;
    private SparkMaxLimitSwitch ARM_reverseLimit;
    private SparkMaxLimitSwitch wrist_forwardLimit;
    private SparkMaxLimitSwitch wrist_reverseLimit;
    private SparkMaxLimitSwitch lClaw_forwardLimit;
    private SparkMaxLimitSwitch lClaw_revLimitSwitch;
    private SparkMaxLimitSwitch RClaw_forwardLimit;
    private SparkMaxLimitSwitch RClaw_revLimitSwitch;
    //create solenoids
    private final Solenoid ARM_Brake;
    private final Solenoid finger;
    private final Solenoid claw;



    public SubArm (int Wrist, int Arm, int lclaw, int rclaw, int ArmBrake, int FingerPnumatic) {

        
// Initialize, Set Id, and restore Factory
        WRIST_MOTOR = new CANSparkMax(Wrist, MotorType.kBrushless);
        WRIST_MOTOR.restoreFactoryDefaults();
// Initialize, Set Id, and restore Factory
        ARM_MOTOR = new CANSparkMax(Arm, MotorType.kBrushless);
        ARM_MOTOR.restoreFactoryDefaults();
// Initialize, Set Id, and restore Factory
        Left_CLAW_MOTOR = new CANSparkMax(lclaw, MotorType.kBrushless);
        Left_CLAW_MOTOR.restoreFactoryDefaults();
//initialize, set id, and restore factory
        RIGHT_CLAW_MOTOR = new CANSparkMax(rclaw, MotorType.kBrushless);
        RIGHT_CLAW_MOTOR.restoreFactoryDefaults();
//set Encoders
        ARM_PIDCONTROLLER = ARM_MOTOR.getPIDController();
        WRIST_PIDCONTROLLER = WRIST_MOTOR.getPIDController();
//set Pnuematics
        ARM_Brake = new Solenoid(PneumaticsModuleType.CTREPCM, ArmBrake);
        finger = new Solenoid(PneumaticsModuleType.CTREPCM, FingerPnumatic);
        claw = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
       


        ARM_ENCODER = ARM_MOTOR.getEncoder();
        WRIST_ENCODER = WRIST_MOTOR.getEncoder();
       

        //set limit switches
        ARM_forwardLimit = ARM_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ARM_reverseLimit = ARM_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        ARM_forwardLimit.enableLimitSwitch(true);
        ARM_reverseLimit.enableLimitSwitch(true);

        wrist_forwardLimit = WRIST_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        wrist_reverseLimit = WRIST_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        wrist_forwardLimit.enableLimitSwitch(true);
        wrist_reverseLimit.enableLimitSwitch(true);

        lClaw_forwardLimit = Left_CLAW_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        lClaw_revLimitSwitch = Left_CLAW_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        lClaw_forwardLimit.enableLimitSwitch(true);
        lClaw_revLimitSwitch.enableLimitSwitch(true);

        RClaw_forwardLimit = RIGHT_CLAW_MOTOR.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        RClaw_revLimitSwitch = RIGHT_CLAW_MOTOR.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        RClaw_forwardLimit.enableLimitSwitch(true);
        RClaw_revLimitSwitch.enableLimitSwitch(true);




        // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 2500;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    //set PID
    ARM_PIDCONTROLLER.setP(kP);
    ARM_PIDCONTROLLER.setI(kI);
    ARM_PIDCONTROLLER.setD(kD);
    ARM_PIDCONTROLLER.setFF(kFF);
    ARM_PIDCONTROLLER.setOutputRange(kMinOutput, kMaxOutput);

    WRIST_PIDCONTROLLER.setP(kP);
    WRIST_PIDCONTROLLER.setI(kI);
    WRIST_PIDCONTROLLER.setD(kD);
    WRIST_PIDCONTROLLER.setFF(kFF);
    WRIST_PIDCONTROLLER.setOutputRange(kMinOutput, kMaxOutput);

    //smart motion coeffeicents
    int smartMotionSlot1 = 0;
    ARM_PIDCONTROLLER.setSmartMotionMaxVelocity(maxVel, smartMotionSlot1);
    ARM_PIDCONTROLLER.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot1);
    ARM_PIDCONTROLLER.setSmartMotionMaxAccel(maxAcc, smartMotionSlot1);
    ARM_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot1);

    WRIST_PIDCONTROLLER.setSmartMotionMaxVelocity(maxVel, smartMotionSlot1);
    WRIST_PIDCONTROLLER.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot1);
    WRIST_PIDCONTROLLER.setSmartMotionMaxAccel(maxAcc, smartMotionSlot1);
    WRIST_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot1);

        

        //create smart dash values
        SmartDashboard.putNumber("Arm Output", ArmPercentOut);
        SmartDashboard.putNumber("Wrist Output", WristPercentOut);
        SmartDashboard.putBoolean("controller Mode", false);
        SmartDashboard.putNumber("Set Arm Position", 0);
        SmartDashboard.putNumber("Set Wrist Position", 0);
        SmartDashboard.putBoolean("Manual Position", false);

        
       
        
        
    }
    
    //Smart Motion mode
    public void runArmSmartMotoion(double setPointA, double setPointW, Boolean fingerPostion) {
        //local variables created at top
        double armEncoderPosition, wristEncoderPosition;
        boolean manual_mode;
        //runs intake motors
        double speed;
       
        if (IO.getLeftBumper()==true) {
                speed = 0.5;
        } else if (IO.getRightBumper()==true) {
                speed = -0.5;
        } else {
                speed = 0;
        }
        claw.set(IO.getAbutton());

        Left_CLAW_MOTOR.set(speed);
        RIGHT_CLAW_MOTOR.set(speed);

        //manual mode toggle getter (must be initialized prior to if else toggle)
         manual_mode = SmartDashboard.getBoolean("Manual Position", false);


        // manual mode allows the positions to be set from the dash while the other mode uses set positions
        if(manual_mode == true) {
        //dash values for positions
        setPointA = SmartDashboard.getNumber("Set Arm Position", 0);
        setPointW = SmartDashboard.getNumber("Set Wrist Position", 0);


        

        //sets motors to smart motion and gives a position
        ARM_PIDCONTROLLER.setReference(setPointA, CANSparkMax.ControlType.kSmartMotion);
        WRIST_PIDCONTROLLER.setReference(setPointW, CANSparkMax.ControlType.kSmartMotion);
        } else {
                //hard coded position, adjustable in Robot.teleopPeriodic
                ARM_PIDCONTROLLER.setReference(setPointA, CANSparkMax.ControlType.kSmartMotion);
                WRIST_PIDCONTROLLER.setReference(setPointW, CANSparkMax.ControlType.kSmartMotion);
        }

//arm brake logic
        if (ARM_ENCODER.getPosition() == setPointA && WRIST_ENCODER.getPosition() == setPointW) {
                ARM_Brake.set(true);
        }  else {
                ARM_Brake.set(false);
        }
        

//Finger Position
        if (fingerPostion == true) {
                finger.set(true);
        } else {
                finger.set(false);
        }


        // zero setters.
        if (ARM_reverseLimit.isPressed() == true) {
                ARM_ENCODER.setPosition(0);
        } else {
                
        }
        if (wrist_reverseLimit.isPressed() == true) {
                WRIST_ENCODER.setPosition(0);
        } else {
                
        }

//Dash related stuff

        //gets encoder values to be put on dash (documentation and bug testing primarily)
        armEncoderPosition = ARM_ENCODER.getPosition();
        wristEncoderPosition = WRIST_ENCODER.getPosition();
        
        //arm dash values
        SmartDashboard.putNumber("SetPointArm", setPointA);
        SmartDashboard.putNumber("Arm encoder", armEncoderPosition);
        SmartDashboard.putNumber("Arm Current in amps ", ARM_MOTOR.getOutputCurrent());
        //wrist dash values
        SmartDashboard.putNumber("SetPointWrist", setPointW);
        SmartDashboard.putNumber("wrist encoder", wristEncoderPosition);
        SmartDashboard.putNumber("Wrist current in amps", WRIST_MOTOR.getOutputCurrent());
        //arm limit switch
        SmartDashboard.putBoolean("Forward A Limit Switch", ARM_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse A Limit Switch", ARM_reverseLimit.isPressed());
        //wrist limit switch
        SmartDashboard.putBoolean("Forward W Limit Switch", wrist_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse W Limit Switch", wrist_reverseLimit.isPressed());
    }
}
