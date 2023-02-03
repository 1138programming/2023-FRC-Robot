// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoring;
public class scoreObject extends CommandBase {
  Scoring scoring = new Scoring();

  public scoreObject(Scoring scoring) {
    this.scoring = scoring;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(true /*if limelight says we're ready*/) scoring.moveClawMotors(0); //establish a constant for this   
  }

  
  @Override
  public void execute() //if the game object has been picked up, claw closes
  {
    
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
