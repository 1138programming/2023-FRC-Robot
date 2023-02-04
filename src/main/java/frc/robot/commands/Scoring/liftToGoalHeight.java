// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import frc.robot.subsystems.Scoring;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class liftToGoalHeight extends CommandBase {
  /** Creates a new liftToGoalHeight. */
  

  Scoring scoring = new Scoring();

  public liftToGoalHeight(Scoring scoring) {
    this.scoring = scoring;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    scoring.moveClawMotors(0); //establish a constant for this   
  }

  
  @Override
  public void execute() //if the game object has been picked up, claw closes
  {
    if(true/*establish if game object has been picked up*/){
        scoring.moveClawMotors(0); //establish another constant for this. only use if the game object hasn't been picked up
        scoring.moveAngleArmMotor(0); //establish a constant for this
    }
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
