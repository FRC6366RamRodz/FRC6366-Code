// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.IO;
import frc.robot.util.Constants.DT_PIDF;
import frc.robot.util.Constants.DT_Set;

/** Add your docs here. */
public class SubDriveTrain {
    private final Timer errTimer = new Timer();
    private final WPI_TalonFX LEFT_FRONT;
    private final WPI_TalonFX LEFT_REAR;
    private final WPI_TalonFX RIGHT_REAR;
    private final WPI_TalonFX RIGHT_FRONT;  
    private final WPI_Pigeon2 Pidgy;
    private double pErr;
    double Padjust;
    double Dadjust;
    private final PIDController turnController;
    static final double kP = 0.003;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 2.0f;
    double rotateToAngleRate;


    public SubDriveTrain(int leftFront,int leftRear, int rightFront,int rightRear){
        turnController = new PIDController(kP, kI, kD);
        turnController.setTolerance(kToleranceDegrees);
        Pidgy = new WPI_Pigeon2(0); 
        rotateToAngleRate = 0;


        LEFT_FRONT = new WPI_TalonFX(leftFront);
        LEFT_REAR = new WPI_TalonFX(leftRear);
        RIGHT_REAR = new WPI_TalonFX(rightRear);
        RIGHT_FRONT = new WPI_TalonFX(rightFront);

        LEFT_FRONT.configFactoryDefault();
        LEFT_REAR.configFactoryDefault();
        RIGHT_REAR.configFactoryDefault();
        RIGHT_FRONT.configFactoryDefault();

        LEFT_REAR.follow(LEFT_FRONT);
        RIGHT_REAR.follow(RIGHT_FRONT);

        LEFT_FRONT.setInverted(false);
        LEFT_REAR.setInverted(false);
        RIGHT_FRONT.setInverted(true);
        RIGHT_REAR.setInverted(true);

        LEFT_FRONT.setNeutralMode(NeutralMode.Coast);
        LEFT_REAR.setNeutralMode(NeutralMode.Coast);
        RIGHT_REAR.setNeutralMode(NeutralMode.Coast);
        RIGHT_FRONT.setNeutralMode(NeutralMode.Coast);

        LEFT_FRONT.configClosedloopRamp(DT_Set.DT_RAMP_RATE_SECS);
        RIGHT_FRONT.configClosedloopRamp(DT_Set.DT_RAMP_RATE_SECS);

        LEFT_FRONT.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor,
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_TimeoutMs);

        RIGHT_FRONT.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor,
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_TimeoutMs);    
            
        LEFT_FRONT.configNominalOutputForward(0, DT_PIDF.DT_TimeoutMs);
        LEFT_FRONT.configNominalOutputReverse(0, DT_PIDF.DT_TimeoutMs);
        LEFT_FRONT.configPeakOutputForward(1, DT_PIDF.DT_TimeoutMs);
        LEFT_FRONT.configPeakOutputReverse(-1, DT_PIDF.DT_TimeoutMs);

        RIGHT_FRONT.configNominalOutputForward(0, DT_PIDF.DT_TimeoutMs);
        RIGHT_FRONT.configNominalOutputReverse(0, DT_PIDF.DT_TimeoutMs);
        RIGHT_FRONT.configPeakOutputForward(1, DT_PIDF.DT_TimeoutMs);
        RIGHT_FRONT.configPeakOutputReverse(-1, DT_PIDF.DT_TimeoutMs);

        LEFT_FRONT.configMotionAcceleration(DT_Set.DT_MAX_VELOCITY);
    //      LEFT_FRONT.configMotionCruiseVelocity(DT_Set.DT_CRUISE_VEL_ENC);

        RIGHT_FRONT.configMotionAcceleration(DT_Set.DT_MAX_VELOCITY);
    //      RIGHT_FRONT.configMotionCruiseVelocity(DT_Set.DT_CRUISE_VEL_ENC);

        LEFT_FRONT.config_kP(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_PG,
            DT_PIDF.DT_TimeoutMs
        );
        LEFT_FRONT.config_kI(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_IG,
            DT_PIDF.DT_TimeoutMs
        );
        LEFT_FRONT.config_kD(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_DG,
            DT_PIDF.DT_TimeoutMs
        );
        LEFT_FRONT.config_kF(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Left_F,
            DT_PIDF.DT_TimeoutMs
        );

        RIGHT_FRONT.config_kP(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Right_PG,
            DT_PIDF.DT_TimeoutMs
        );
        RIGHT_FRONT.config_kI(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Right_IG,
            DT_PIDF.DT_TimeoutMs
        );
        RIGHT_FRONT.config_kD(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Right_DG,
            DT_PIDF.DT_TimeoutMs
        );
        RIGHT_FRONT.config_kF(
            DT_PIDF.DT_PIDLoopIdx,
            DT_PIDF.DT_Right_F,
            DT_PIDF.DT_TimeoutMs
        );

        LEFT_FRONT.configMotionAcceleration(DT_Set.MM_ACCELERATION);
        RIGHT_FRONT.configMotionAcceleration(DT_Set.MM_ACCELERATION);
        LEFT_FRONT.configMotionCruiseVelocity(DT_Set.MM_CRUISECONTROL);
        RIGHT_FRONT.configMotionCruiseVelocity(DT_Set.MM_CRUISECONTROL);
        Pidgy.reset();

    }

    public void runDrive(double forward,double rotate, boolean X, boolean Xp, boolean Xr){  
        // p = -0.035 and d = 0.003 minCmd = 0.035 
        float Kp = -0.048f;
        double Kd = 0.002;
        float min_command = 0.033f;
        double left;
        double right;
       
        if (Xp == true) {
            errTimer.start();
        }
        if (Xr == true) {
            errTimer.stop();
        }

        double heading_error = Pidgy.getPitch();
        if (errTimer.get() == 0.0) {
            pErr = Pidgy.getRoll();
        } 
        if(errTimer.get() > 0.2) {
            errTimer.restart();
        }
        if (X == true)
        {
                
                
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
                left  = Padjust * Dadjust;
                right = Padjust * Dadjust;
        } else {

        double fward = Math.abs(forward);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
            if (forward == 0) {
                fward = 1;
                sens = DT_Set.DT_QUICK_TURN;
            }

            left = -forward + rotate * fward * sens;
            right = -forward - rotate * fward * sens;
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    
            if (maxMagnitude > 1){
                left /= maxMagnitude;
                right /= maxMagnitude;
            }
        }

        LEFT_FRONT.set(ControlMode.Velocity, left * DT_Set.DT_MAX_VELOCITY);
        RIGHT_FRONT.set(ControlMode.Velocity, right * DT_Set.DT_MAX_VELOCITY);
    }
    
    public void runGyroDrive() {
        boolean rotateToAngle = false;
        double PidgyError = Pidgy.getAngle();
        double left, right;

        if (IO.getYbutton()) {
            turnController.setSetpoint(0.0f);
            rotateToAngle = true;
        } else if (IO.getAbutton()) {
            turnController.setSetpoint(90.0f);
            rotateToAngle = true;
        } else if (IO.getBbutton()) {
            turnController.setSetpoint(179.9f);
            rotateToAngle = true;
        } else if (IO.getXbutton()) {
            turnController.setSetpoint(-90.0f);
            rotateToAngle = true;
        }
        rotateToAngleRate = turnController.calculate(PidgyError);
        double currentRotationRate;
        if ( rotateToAngle ) {
            currentRotationRate = rotateToAngleRate;
        } else {
            currentRotationRate = 0;
        }



        left = -0 + currentRotationRate;
        right = -0 - currentRotationRate;

        LEFT_FRONT.set(ControlMode.Velocity, left * DT_Set.DT_MAX_VELOCITY);
        RIGHT_FRONT.set(ControlMode.Velocity, right * DT_Set.DT_MAX_VELOCITY);
    }

    public double getLeftSpeed(){
        return LEFT_FRONT.getSelectedSensorVelocity();
    }

    public double getRightSpeed(){
        return RIGHT_FRONT.getSelectedSensorVelocity();
    }

    public double getLeftPosition(){
        return LEFT_FRONT.getSelectedSensorPosition();
    }

    public double getRightPosition(){
        return RIGHT_FRONT.getSelectedSensorPosition();
    }

    public void resetOdometer(){
        LEFT_FRONT.setSelectedSensorPosition(0);
        RIGHT_FRONT.setSelectedSensorPosition(0);
    }

    public double getRoll(){
        return Pidgy.getPitch();
    }


}







