// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
//create solenoids
    private final Solenoid STINGER_SOLENOID;
    private final Solenoid STRAFE_SOLENOID;

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
        LEFT_FRONT_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, Constants.DT_Set.kMaxOutput);
        RIGHT_FRONT_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, Constants.DT_Set.kMaxOutput);
        STINGER_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, Constants.DT_Set.kMaxOutput);
        STRAFE_PIDCONTROLLER.setOutputRange(DT_Set.kMinOutput, Constants.DT_Set.kMaxOutput);
        // Drive Train acceleration value
        
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
    }
// teleop tank drive
    public void runTankDrive (double leftY, double rightX, boolean limelightButton) {
        LEFT_FRONT_MOTOR.setClosedLoopRampRate(1.7);
        RIGHT_FRONT_MOTOR.setClosedLoopRampRate(1.7);
        STINGER_MOTOR.setClosedLoopRampRate(4.5);
        STRAFE_MOTOR.setClosedLoopRampRate(4.5);
        //Targeting Stuff
        double targetX;
        double left, right;
        targetX =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

        double Kp = -0.0055;
        double min_command = 0.03;
        double heading_error = -targetX;
        double steering_adjust = 0.0;

        STINGER_SOLENOID.set(false);
        STRAFE_SOLENOID.set(false);

        // controlls
        if (limelightButton == true) {//start of limelight code
            if (Math.abs(heading_error) > 0.8) {
                if (heading_error < 0) {
                     steering_adjust = Kp*heading_error + min_command;
                }
                else{
                    steering_adjust = Kp*heading_error - min_command;
                }
                    }
                   left = steering_adjust+(-leftY*0.6);
                   right = -steering_adjust+(-leftY*0.6);       
        } 
        else { //start of regular steering code
            double fward = Math.abs(leftY);
            double sens = DT_Set.DT_TURN_SENSITIVITY;
        if (leftY == 0) {
            fward = 1;
            sens = DT_Set.DT_QUICK_TURN;
    
        }
            left = -leftY + rightX * fward * sens;
            right = -leftY - rightX * fward * sens;
            double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    
        if (maxMagnitude > 1){
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
    }
        //sets speeed
        LEFT_FRONT_PIDCONTROLLER.setReference(left*DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
        RIGHT_FRONT_PIDCONTROLLER.setReference(-right*DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
        STINGER_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kVelocity);
        STRAFE_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kVelocity);
        //gives encoder count


        
    }
//teleop stinger drive
    public void runStingerDrive (double leftY, double leftX, double rightX, double triggerSteer){
        //slew limiter (controller acc limit)
        LEFT_FRONT_MOTOR.setClosedLoopRampRate(1.0);
        RIGHT_FRONT_MOTOR.setClosedLoopRampRate(1.0);
        STINGER_MOTOR.setClosedLoopRampRate(1.0);
        STRAFE_MOTOR.setClosedLoopRampRate(4.5);
        //set Solenoid       
        STINGER_SOLENOID.set(true);
        STRAFE_SOLENOID.set(false);

        // controlls
        double fward = Math.abs(leftY);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
        if (leftY == 0) {
            fward = 1;
            sens = DT_Set.DT_QUICK_TURN;
        }
        double left = -leftY + rightX * fward * sens;
        double right = -leftY - rightX * fward * sens;
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    
        if (maxMagnitude > 1){
            left /= maxMagnitude;
            right /= maxMagnitude;
        }

        //sets speeed
        LEFT_FRONT_PIDCONTROLLER.setReference(left*DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
        RIGHT_FRONT_PIDCONTROLLER.setReference(-right*DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
        STINGER_PIDCONTROLLER.setReference(triggerSteer*DT_Set.maxRPM, CANSparkMax.ControlType.kVelocity);
        STRAFE_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kVelocity);
    }
// teleop strafe drive
    public void runStrafeDrive (double leftX, double rightX, double triggerSteer) {
        //slew limiter (controller acc limit)
        LEFT_FRONT_MOTOR.setClosedLoopRampRate(1.7);
        RIGHT_FRONT_MOTOR.setClosedLoopRampRate(1.7);
        STINGER_MOTOR.setClosedLoopRampRate(4.5);
        STRAFE_MOTOR.setClosedLoopRampRate(4.5);
        //solenoid set
        STINGER_SOLENOID.set(true);
        STRAFE_SOLENOID.set(true);

        // controlls
        double fward = Math.abs(leftX);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
        if (leftX == 0) {
            fward = 1;
            sens = DT_Set.DT_QUICK_TURN;
        }
        double left = -leftX + rightX * fward * sens;
        double right = -leftX - rightX * fward * sens;
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    
        if (maxMagnitude > 1){
            left /= maxMagnitude;
            right /= maxMagnitude;
        }

        //sets speeed
        LEFT_FRONT_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kVelocity);
        RIGHT_FRONT_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kVelocity);
        STINGER_PIDCONTROLLER.setReference(left*DT_Set.MaxStrRPM, CANSparkMax.ControlType.kVelocity);
        STRAFE_PIDCONTROLLER.setReference(-right*DT_Set.MaxStrRPM, CANSparkMax.ControlType.kVelocity);
    }
//autonomous Tank Drive
    public void autoTankDrive (double leftEncoder, double rightEncoder) {
        //reset strafe motors
        STINGER_ENCODER.setPosition(0);
        STRAFE_ENCODER .setPosition(0);
        //solenoids
        STINGER_SOLENOID.set(false);
        STRAFE_SOLENOID.set(false);
        //motors
        LEFT_FRONT_PIDCONTROLLER.setReference(leftEncoder, CANSparkMax.ControlType.kSmartMotion);
        RIGHT_FRONT_PIDCONTROLLER.setReference(-rightEncoder, CANSparkMax.ControlType.kSmartMotion);
        STINGER_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        STRAFE_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }
//autonomous strafe drive
    public void autoStrafeDrive (double frontEncoder, double rearEncoder) {
                //reset drive motors
                LEFT_FRONT_ENCODER.setPosition(0);
                RIGHT_FRONT_ENCODER.setPosition(0);
                //solenoids
                STINGER_SOLENOID.set(true);
                STRAFE_SOLENOID.set(true);
                //motors
                LEFT_FRONT_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
                RIGHT_FRONT_PIDCONTROLLER.setReference(0, CANSparkMax.ControlType.kSmartMotion);
                STINGER_PIDCONTROLLER.setReference(rearEncoder, CANSparkMax.ControlType.kSmartMotion);
                STRAFE_PIDCONTROLLER.setReference(frontEncoder, CANSparkMax.ControlType.kSmartMotion);
    }
// resets encoders
    public void resetDriveEncoder() {
        LEFT_FRONT_ENCODER.setPosition(0);
        RIGHT_FRONT_ENCODER.setPosition(0);
        STINGER_ENCODER.setPosition(0);
        STRAFE_ENCODER .setPosition(0);
    }
// Speed tuning testing basic percent output no steering
    public void runPercent(double leftX) {
        LEFT_FRONT_MOTOR.set(-leftX);
        RIGHT_FRONT_MOTOR.set(leftX);


    }
//dash stuff
    public double getLeftDriveTemp() {
        return LEFT_FRONT_MOTOR.getMotorTemperature();
    }

    public double getRightDriveTemp() {
        return RIGHT_FRONT_MOTOR.getMotorTemperature();
    }

    public double getLEFTencoder() {
        return LEFT_FRONT_ENCODER.getPosition();
    }

    public double getRIGHTencoder() {
        return RIGHT_FRONT_ENCODER.getPosition();
    }

    public double getSTRAFEencoder() {
        return STRAFE_ENCODER.getPosition();
    }

    public double getSTINGERencoder() {
        return STINGER_ENCODER.getPosition();
    }

    public void resetEncoders() {
        LEFT_FRONT_ENCODER.setPosition(0);
        RIGHT_FRONT_ENCODER.setPosition(0);
        STINGER_ENCODER.setPosition(0);
        STRAFE_ENCODER.setPosition(0);
    }

    public double getLeftSpeed() {
        return LEFT_FRONT_ENCODER.getVelocity();
    }

    public double getRightSpeed() {
        return RIGHT_FRONT_ENCODER.getVelocity();
    }
    }

