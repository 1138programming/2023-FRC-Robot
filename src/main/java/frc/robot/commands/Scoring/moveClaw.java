// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoring;
public class MoveClaw extends CommandBase {
  Scoring scoring;
  double position;
  public MoveClaw(Scoring scoring, double position) {
    this.scoring = scoring;
    this.position = position;
    addRequirements(scoring);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() 
  {
    scoring.moveClaw(position);
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
