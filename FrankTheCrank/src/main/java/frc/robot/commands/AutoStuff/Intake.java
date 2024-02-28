// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoStuff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Intake extends Command {
  /** Creates a new Intake. */
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.run3PointArm(true, false, false, false, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.run3PointArm(false, false, false, false, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.shooter.IntakeRumble() == 1) {
      return true;
    }
    return false;
  }
}
