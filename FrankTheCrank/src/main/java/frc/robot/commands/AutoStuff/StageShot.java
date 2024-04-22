// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoStuff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StageShot extends Command {
  public Timer shotTime = new Timer();
  /** Creates a new StageShot. */
  public StageShot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotTime.stop();
    shotTime.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.shooter.LaunchPermision() == 1) {
      shotTime.start();
      RobotContainer.shooter.advancedShoot(false, false, false, true, false, false, false, true, 0, false, false, 0);
    } else {
      RobotContainer.shooter.advancedShoot(false, false, false, true, false, false, false, false, 0, false, false, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.advancedShoot(false, false, false, false, false, false, false, false, 0, false, false, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shotTime.get() > 1.5) {
      return true;
    } else {
      return false;
    }
  }
}
