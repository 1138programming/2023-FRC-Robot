// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.subsystems.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WriteOdometryLog extends CommandBase {
  private Base base;
  /** Creates a new WriteOdometryLog. */
  public WriteOdometryLog(Base base) {
    this.base = base;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.writeOdometryData();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
