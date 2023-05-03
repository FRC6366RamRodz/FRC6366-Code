// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.DT_Set;


/** Add your docs here. */
public class SubRRDrive {
    private final WPI_TalonSRX LEFT_FRONT;
    private final WPI_TalonSRX RIGHT_FRONT;
    private final WPI_VictorSPX LEFT_REAR;
    private final WPI_VictorSPX RIGHT_REAR;
    private final CANSparkMax STRAFE_MOTOR;
    private final CANSparkMax STINGER_MOTOR;
    private final Solenoid STINGER_SOLENOID;
    private final Solenoid STRAFE_SOLENOID;


    public SubRRDrive(int leftfront, int righfront, int leftrear, int rightrear, int strafeMotor, int stingerMotor, boolean toggle, int stingersolenoid, int strafesolenoid) {
        
        LEFT_FRONT = new WPI_TalonSRX(leftfront);
        RIGHT_FRONT = new WPI_TalonSRX(righfront);
        LEFT_REAR = new WPI_VictorSPX(leftrear);
        RIGHT_REAR = new WPI_VictorSPX(rightrear);
        STRAFE_MOTOR = new CANSparkMax(strafeMotor, MotorType.kBrushless);
        STINGER_MOTOR = new CANSparkMax(stingerMotor, MotorType.kBrushless);

        LEFT_FRONT.configFactoryDefault();
        RIGHT_FRONT.configFactoryDefault();
        LEFT_REAR.configFactoryDefault();
        RIGHT_REAR.configFactoryDefault();
        STRAFE_MOTOR.restoreFactoryDefaults();
        STINGER_MOTOR.restoreFactoryDefaults();

        

        LEFT_FRONT.setInverted(false);
        RIGHT_FRONT.setInverted(false);
        LEFT_REAR.setInverted(false);
        RIGHT_REAR.setInverted(false);
        STRAFE_MOTOR.setInverted(false);
        STINGER_MOTOR.setInverted(false);

       
        LEFT_REAR.follow(LEFT_FRONT);
        RIGHT_REAR.follow(RIGHT_FRONT);
        
        STINGER_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, stingersolenoid);
        STRAFE_SOLENOID = new Solenoid(PneumaticsModuleType.REVPH, strafesolenoid);


        SmartDashboard.putBoolean("SlowMode", toggle);

    }

   
    public void runTankdrive (double leftY, double steering, boolean quickTurn, boolean SlowMode) {
        
        STINGER_SOLENOID.set(false);
        STRAFE_SOLENOID.set(false);

        double fward = Math.abs(leftY);
        double rotate = fward * steering;
        if (Math.abs(leftY) == 0) {
            fward = 0.6;
        } 

        double forward = leftY;
        if (SlowMode = true) {
             forward = leftY * DT_Set.DT_slowSpeed;
  
        }

        LEFT_FRONT.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +rotate);
        RIGHT_FRONT.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -rotate);
        STINGER_MOTOR.set(0);
        STRAFE_MOTOR.set(0);

    }

    public void runStingerDrive (double leftY, double steering, double hSteer) {

        STINGER_SOLENOID.set(true);
        STRAFE_SOLENOID.set(false);

        LEFT_FRONT.set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, - steering);
        RIGHT_FRONT.set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, + steering);
        STINGER_MOTOR.set(hSteer);
        STRAFE_MOTOR.set(0);

    }

    public void runStrafeDrive (double leftX, double hSteer) {

        double fward = Math.abs(leftX);
        double sens = DT_Set.DT_TURN_SENSITIVITY;
        if (leftX == 0) {
            fward = 1;
        } 

        STINGER_SOLENOID.set(true);
        STRAFE_SOLENOID.set(true);

        double left = -leftX + hSteer * fward * sens;
        double right = -leftX - hSteer * fward * sens;
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
    
        if (maxMagnitude > 1){
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
        STINGER_MOTOR.set(left);
        STRAFE_MOTOR.set(right);
        
    }

}
