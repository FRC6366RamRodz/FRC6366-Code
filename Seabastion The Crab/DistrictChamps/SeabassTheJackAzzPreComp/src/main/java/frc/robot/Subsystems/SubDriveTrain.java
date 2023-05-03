// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.DT_Set;

/** Add your docs here. */
public class SubDriveTrain {
    // create drive train motors and assorted crap
    private SparkMaxPIDController LEFT_FRONT_PIDCONTROLLER;
    private SparkMaxPIDController RIGHT_FRONT_PIDCONTROLLER;
    private SparkMaxPIDController STINGER_PIDCONTROLLER;
    private SparkMaxPIDController STRAFE_PIDCONTROLLER;
    private RelativeEncoder LEFT_FRONT_ENCODER;
    private RelativeEncoder RIGHT_FRONT_ENCODER;
    private RelativeEncoder STINGER_ENCODER;
    private RelativeEncoder STRAFE_ENCODER;
    private CANSparkMax LEFT_FRONT_MOTOR;
    private CANSparkMax RIGHT_FRONT_MOTOR;
    private CANSparkMax LEFT_REAR_MOTOR;
    private CANSparkMax RIGHT_REAR_MOTOR;
    private CANSparkMax STINGER_MOTOR;
    private CANSparkMax STRAFE_MOTOR;
    private WPI_Pigeon2 Pidgy;
    private PIDController BalancePid;
//create solenoids
    private final Solenoid STINGER_SOLENOID;
    private final Solenoid STRAFE_SOLENOID;
    double Padjust;
    double Dadjust;
    private final PIDController turnController;
    static final double kP = 0.003;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 0.0f;
    double rotateToAngleRate;

    /* ease of coding paste for motors 
    LEFT_FRONT_MOTOR
        LEFT_REAR_MOTOR
        RIGHT_FRONT_MOTOR
        RIGHT_REAR_MOTOR
        STINGER_MOTOR
        STRAFE_MOTOR 
        
        LEFT_FRONT_PIDCONTROLLER
        RIGHT_FRONT_PIDCONTROLLER
        STINGER_PIDCONTROLLER
        STRAFE_PIDCONTROLLER
        */

    public SubDriveTrain(int leftFront, int rightFront, int leftRear, int rigthRear, int stinger, int strafe, int stingerSolenoid, int strafeSolenoid) {
        BalancePid = new PIDController(-0.005, 0, 0);
        Pidgy = new WPI_Pigeon2(0);
         turnController = new PIDController(kP, kI, kD);
        turnController.setTolerance(kToleranceDegrees);
        rotateToAngleRate = 0;
        //set motor's
        LEFT_FRONT_MOTOR = new CANSparkMax(leftFront, MotorType.kBrushless);
        LEFT_REAR_MOTOR = new CANSparkMax(leftRear, MotorType.kBrushless);
        RIGHT_FRONT_MOTOR = new CANSparkMax(rightFront, MotorType.kBrushless);
        RIGHT_REAR_MOTOR = new CANSparkMax(rigthRear, MotorType.kBrushless);
        STINGER_MOTOR = new CANSparkMax(stinger, MotorType.kBrushless);
        STRAFE_MOTOR = new CANSparkMax(strafe, MotorType.kBrushless);
        // set solenoids
        STINGER_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, stingerSolenoid);
        STRAFE_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, strafeSolenoid);

        LEFT_FRONT_PIDCONTROLLER = LEFT_FRONT_MOTOR.getPIDController();
        RIGHT_FRONT_PIDCONTROLLER = RIGHT_FRONT_MOTOR.getPIDController();
        STINGER_PIDCONTROLLER = STINGER_MOTOR.getPIDController();
        STRAFE_PIDCONTROLLER = STRAFE_MOTOR.getPIDController();

        //factory defaults
        LEFT_FRONT_MOTOR.restoreFactoryDefaults();
        LEFT_REAR_MOTOR.restoreFactoryDefaults();
        RIGHT_FRONT_MOTOR.restoreFactoryDefaults();
        RIGHT_REAR_MOTOR.restoreFactoryDefaults();
        STINGER_MOTOR.restoreFactoryDefaults();
        STRAFE_MOTOR.restoreFactoryDefaults();
        //encoders
        LEFT_FRONT_ENCODER = LEFT_FRONT_MOTOR.getEncoder();
        RIGHT_FRONT_ENCODER = RIGHT_FRONT_MOTOR.getEncoder();
        STINGER_ENCODER = STINGER_MOTOR.getEncoder();
        STRAFE_ENCODER = STRAFE_MOTOR.getEncoder();
        
        // set P
        LEFT_FRONT_PIDCONTROLLER.setP(DT_Set.kP);
        RIGHT_FRONT_PIDCONTROLLER.setP(DT_Set.kP);
        STINGER_PIDCONTROLLER.setP(DT_Set.kP);
        STRAFE_PIDCONTROLLER.setP(DT_Set.kP);
        // set I
        LEFT_FRONT_PIDCONTROLLER.setI(DT_Set.kI);
        RIGHT_FRONT_PIDCONTROLLER.setI(DT_Set.kI);
        STINGER_PIDCONTROLLER.setI(DT_Set.kI);
        STRAFE_PIDCONTROLLER.setI(DT_Set.kI);
        // set D
        LEFT_FRONT_PIDCONTROLLER.setD(DT_Set.kD);
        RIGHT_FRONT_PIDCONTROLLER.setD(DT_Set.kD);
        STINGER_PIDCONTROLLER.setD(DT_Set.kD);
        STRAFE_PIDCONTROLLER.setD(DT_Set.kD);
        // set FF
        LEFT_FRONT_PIDCONTROLLER.setFF(DT_Set.kFF);
        RIGHT_FRONT_PIDCONTROLLER.setFF(DT_Set.kFF);
        STINGER_PIDCONTROLLER.setFF(DT_Set.kFF);
        STRAFE_PIDCONTROLLER.setFF(DT_Set.kFF);
        // Max/Min Outputs
        LEFT_FRONT_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, DT_Set.kMaxOutput);
        RIGHT_FRONT_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, DT_Set.kMaxOutput);
        STINGER_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, DT_Set.kMaxOutput);
        STRAFE_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, DT_Set.kMaxOutput);        
        //Smart Motion Max Velocity
        int smartMotionSlot = 0;
        LEFT_FRONT_PIDCONTROLLER.setSmartMotionMaxVelocity(DT_Set.maxVel , smartMotionSlot);
        RIGHT_FRONT_PIDCONTROLLER.setSmartMotionMaxVelocity(DT_Set.maxVel , smartMotionSlot);
        STINGER_PIDCONTROLLER.setSmartMotionMaxVelocity(DT_Set.maxVel , smartMotionSlot);
        STRAFE_PIDCONTROLLER.setSmartMotionMaxVelocity(DT_Set.maxVel , smartMotionSlot);
        //Smart Motion Min Velocity
        LEFT_FRONT_PIDCONTROLLER.setSmartMotionMinOutputVelocity(DT_Set.minVel, smartMotionSlot);
        RIGHT_FRONT_PIDCONTROLLER.setSmartMotionMinOutputVelocity(DT_Set.minVel, smartMotionSlot);
        STINGER_PIDCONTROLLER.setSmartMotionMinOutputVelocity(DT_Set.minVel, smartMotionSlot);
        STRAFE_PIDCONTROLLER.setSmartMotionMinOutputVelocity(DT_Set.minVel, smartMotionSlot);
        //Smart Motion Max acceleration
        LEFT_FRONT_PIDCONTROLLER.setSmartMotionMaxAccel(DT_Set.maxAcc, smartMotionSlot);
        RIGHT_FRONT_PIDCONTROLLER.setSmartMotionMaxAccel(DT_Set.maxAcc, smartMotionSlot);
        STINGER_PIDCONTROLLER.setSmartMotionMaxAccel(DT_Set.maxAcc, smartMotionSlot);
        STRAFE_PIDCONTROLLER.setSmartMotionMaxAccel(DT_Set.maxAcc, smartMotionSlot);
        //Smart Motion Min acceleration
        LEFT_FRONT_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(DT_Set.allowedErr, smartMotionSlot);
        RIGHT_FRONT_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(DT_Set.allowedErr, smartMotionSlot);
        STINGER_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(DT_Set.allowedErr, smartMotionSlot);
        STRAFE_PIDCONTROLLER.setSmartMotionAllowedClosedLoopError(DT_Set.allowedErr, smartMotionSlot);
        //look into motion accel stratagey
        //set inverts
        LEFT_REAR_MOTOR.setInverted(false);
        RIGHT_FRONT_MOTOR.setInverted(false);
        RIGHT_REAR_MOTOR.setInverted(false);
        STINGER_MOTOR.setInverted(false);
        STRAFE_MOTOR.setInverted(false);
        //follow The drive motors
        LEFT_REAR_MOTOR.follow(LEFT_FRONT_MOTOR);
        RIGHT_REAR_MOTOR.follow(RIGHT_FRONT_MOTOR);
        //smart Dashboard
        LEFT_FRONT_MOTOR.burnFlash();
        LEFT_REAR_MOTOR.burnFlash();
        RIGHT_FRONT_MOTOR.burnFlash();
        RIGHT_REAR_MOTOR.burnFlash();
        STINGER_MOTOR.burnFlash();
        STRAFE_MOTOR.burnFlash();
    }
// teleop drive 
    public void runDriveTrain (double left, double right, double strafe, double stinger, boolean strafeButterfly, boolean stingerButterfly, double tankAcc, double strafeAcc, double stingerAcc, boolean limelightButton, double leftY, double leftX, boolean slowMode) {
        double Left, Right, Strafe, Stinger, Left2, Right2, Strafe2, Stinger2;
        double targetX;
        targetX =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        //tank
        double Kp = -0.0045;
        double min_command = 0.01;
        double heading_error = -targetX;
        double steering_adjust = 0.0;
        //strafe
        double Kp1 = -0.0045;
        double min_command1 = 0.08;
        double heading_error1 = -targetX;
        double steering_adjust1 = 0.0;
        if (limelightButton == true && strafeButterfly == false) {//start of limelight code
            if (Math.abs(heading_error) > 0.5) {
                if (heading_error < 0) {
                     steering_adjust = Kp*heading_error + min_command;
                }
                else{
                    steering_adjust = Kp*heading_error - min_command;
                }
                    }
                   Left = steering_adjust+(-leftY*0.6);
                   Right = -steering_adjust+(-leftY*0.6);       
        } else {
            Left = left;
            Right = right;
        } 
        if (limelightButton == true && strafeButterfly == true) {
            if (Math.abs(heading_error1) > 0.5) {
                if (heading_error1 < 0) {
                     steering_adjust1 = Kp*heading_error1 + min_command1;
                }
                else{
                    steering_adjust1 = Kp1*heading_error1 - min_command1;
                }
                    }
                   Strafe = -steering_adjust1+(-leftX*0.6);
                   Stinger = -steering_adjust1+(-leftX*0.6);       
        } else {
            Strafe = strafe;
            Stinger = stinger;
        }  
        if (slowMode == true && stingerButterfly == true && strafeButterfly == false) {
            Left2 = Left*0.4;
            Right2 = Right*0.4;
            Strafe2 = Strafe*0.2;
            Stinger2 = Stinger*0.5;
        }  else if (slowMode==true) {
            Left2 = Left*0.2;
            Right2 = Right*0.2;
            Strafe2 = Strafe*0.2;
            Stinger2 = Stinger*0.2;
        } else {
            Left2 = Left;
            Right2 = Right;
            Strafe2 = Strafe;
            Stinger2 = Stinger;
        }
        //set drive motors
        LEFT_FRONT_PIDCONTROLLER.setReference(Left2*Constants.AR_SET.maxRPM, CANSparkMax.ControlType.kVelocity);
        RIGHT_FRONT_PIDCONTROLLER.setReference(Right2*-Constants.AR_SET.maxRPM, CANSparkMax.ControlType.kVelocity);
        STRAFE_PIDCONTROLLER.setReference(-Strafe2*10.0, CANSparkMax.ControlType.kVoltage);
        STINGER_PIDCONTROLLER.setReference(Stinger2*10.0, CANSparkMax.ControlType.kVoltage);
        //butturfly control
        STRAFE_SOLENOID.set(strafeButterfly);
        STINGER_SOLENOID.set(stingerButterfly);
        //acc limiter
        LEFT_FRONT_MOTOR.setClosedLoopRampRate(tankAcc);
        RIGHT_FRONT_MOTOR.setClosedLoopRampRate(tankAcc);
        STRAFE_MOTOR.setOpenLoopRampRate(strafeAcc);
        STINGER_MOTOR.setOpenLoopRampRate(stingerAcc);
        
    }
//auto drive
    public void runTankAuto (double left, double right, boolean balance) {
   /*      float Kp = -0.048f;
        double Kd = 0.002;
        double min_command = 0.20;
        double Left;
        double Right;
        double heading_error = Pidgy.getPitch();
        if (errTimer.get() == 0.0) {
            pErr = Pidgy.getRoll();
        } 
        if(errTimer.get() > 0.2) {
            errTimer.restart();
        }
        
                
       if (balance == true) {
      
        if (Math.abs(heading_error) > 1.3)
                {
                    
                        if (heading_error < 0)
                        {
                                Padjust = Kp*heading_error - min_command;
                                Dadjust = Kd *(pErr+heading_error)/0.2;
                        }
                        else
                        {
                                Padjust =Kp*heading_error + min_command;
                                Dadjust =Kd *(pErr-heading_error)/0.2;
                        }
                } else {
                    Padjust = 0;
                    Dadjust = 0;
                }
                Left  = Padjust * Dadjust;
                Right = Padjust * Dadjust;
                LEFT_FRONT_PIDCONTROLLER.setReference(Left*Constants.DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
                RIGHT_FRONT_PIDCONTROLLER.setReference(-Right*Constants.DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
       } else {
        LEFT_FRONT_PIDCONTROLLER.setReference(left, CANSparkMax.ControlType.kSmartMotion);
        RIGHT_FRONT_PIDCONTROLLER.setReference(-right, CANSparkMax.ControlType.kSmartMotion);
       } */
 
       double HeadingError = Pidgy.getPitch();
       //error is the output
       double PIDout =  BalancePid.calculate(HeadingError);
       double Left, Right;
       if (balance == true) {
         Right = PIDout;
         Left = PIDout;
         LEFT_FRONT_PIDCONTROLLER.setReference(Left*Constants.DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
         RIGHT_FRONT_PIDCONTROLLER.setReference(-Right*Constants.DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
       } else {
        Right = right;
        Left = left;
        LEFT_FRONT_PIDCONTROLLER.setReference(Left, CANSparkMax.ControlType.kSmartMotion);
       RIGHT_FRONT_PIDCONTROLLER.setReference(Right, CANSparkMax.ControlType.kSmartMotion);
       }
       
       
       
        STRAFE_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        STINGER_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        //butturfly control
        STRAFE_SOLENOID.set(false);
        STINGER_SOLENOID.set(false);
    }

    public void runGyroDrive(boolean rotateToAngle, double setpoint) { 
        double PidgyError = Pidgy.getAngle();
        double left, right;

        turnController.setSetpoint(setpoint);

        
        rotateToAngleRate = turnController.calculate(PidgyError);
        double currentRotationRate;
        if ( rotateToAngle==true) {
            currentRotationRate = rotateToAngleRate;
        } else {
            currentRotationRate = 0;
        }


        left = -0 + currentRotationRate;
        right = -0 + currentRotationRate;

        LEFT_FRONT_PIDCONTROLLER.setReference(left*Constants.DT_Set.maxRPM, ControlType.kVelocity);
        RIGHT_FRONT_PIDCONTROLLER.setReference(right*Constants.DT_Set.maxRPM, ControlType.kVelocity);

    }
//get encoders
    public double getLeftDriveEncoder() {
        return LEFT_FRONT_ENCODER.getPosition();
    }

    public double getLeftDriveSpeed() {
        return LEFT_FRONT_ENCODER.getVelocity();
    }

    public double getRightDriveEncoder() {
        return RIGHT_FRONT_ENCODER.getPosition();
    }

    public double getRightDriveSpeed() {
        return RIGHT_FRONT_ENCODER.getVelocity();
    }

    public double getStingerEncoder() {
        return STINGER_ENCODER.getPosition();
    }

    public double getStrafeEncoder() {
        return STRAFE_ENCODER.getPosition();
    }
//get drive temps
    public double getLeftDriveTemp() {
        return LEFT_FRONT_MOTOR.getMotorTemperature();
    }
    public double getRightDriveTemp() {
        return RIGHT_FRONT_MOTOR.getMotorTemperature();
    }

    public void zeroDrive() {
        LEFT_FRONT_ENCODER.setPosition(0);
        RIGHT_FRONT_ENCODER.setPosition(0);
        STRAFE_ENCODER.setPosition(0);
        STINGER_ENCODER.setPosition(0);
    }

    public void zeroGyro() {
        Pidgy.reset();
    }
}
