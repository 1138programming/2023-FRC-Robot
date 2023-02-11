// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoring;
public class moveClaw extends CommandBase {
  Scoring scoring = new Scoring();
  double speed;
  public moveClaw(Scoring scoring, double speed) {
    this.scoring = scoring;
    this.speed = speed;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() 
  {
    scoring.moveClaw(speed);
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
