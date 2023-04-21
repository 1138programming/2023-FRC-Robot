// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
public class ToggleSpeed extends CommandBase {
  private Base base;
//   private 
private double driveSpeedFactor;
private double rotSpeedFactor;

  public ToggleSpeed(Base base, double driveSpeedFactor, double rotSpeedFactor) {
    this.base = base;
    this.driveSpeedFactor = driveSpeedFactor;
    this.rotSpeedFactor = rotSpeedFactor;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    base.setDriveSpeedFactor(driveSpeedFactor);
    base.setRotSpeedFactor(rotSpeedFactor);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
