// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;
public class ToggleSpeed extends CommandBase {
private Base base;
private double speedFactor;

  public ToggleSpeed(Base base, double speedFactor) {
    this.base = base;
    this.speedFactor = speedFactor;
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    base.setDriveSpeedFactor(speedFactor);
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
