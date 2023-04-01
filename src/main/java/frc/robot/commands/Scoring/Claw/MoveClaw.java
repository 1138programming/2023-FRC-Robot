// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
public class MoveClaw extends CommandBase {
  Claw claw;
  double position;
  public MoveClaw(Claw claw, double position) {
    this.claw = claw;
    this.position = position;
    addRequirements(claw);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() 
  {
    claw.moveClaw(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
