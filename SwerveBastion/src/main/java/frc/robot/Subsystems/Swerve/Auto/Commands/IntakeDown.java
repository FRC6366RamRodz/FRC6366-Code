// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.Auto.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class IntakeDown extends CommandBase {
  public Timer time = new Timer();
  
  /** Creates a new IntakeDown. */
  public IntakeDown() {


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){ 
    time.reset();
    time.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time.start();

    if (time.get()<0.5) {
      RobotContainer.arm.AutoMode(88, 67, 0, false, true);
    } else if (time.get()<1) {
      RobotContainer.arm.AutoMode(88, 67, 0.4, false, true);
    } else if (time.get()<2.8) {
      RobotContainer.arm.AutoMode(88, 67, 0, false, false);
    } else if (time.get()<4) {
      RobotContainer.arm.AutoMode(99, 120, -0.4, false, false);
    } else if (time.get()<5) {
      RobotContainer.arm.AutoMode(178, 156, 0, false, true);
    } else {
      RobotContainer.arm.AutoMode(178, 156, 0.3, false, true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
