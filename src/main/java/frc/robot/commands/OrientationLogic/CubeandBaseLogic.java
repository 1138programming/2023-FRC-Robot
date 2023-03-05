// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OrientationLogic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Orientation.*;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Scoring;
import static frc.robot.Constants.*;

public class CubeandBaseLogic extends CommandBase {
  /** Creates a new OrientationMove. */

  private Orientation orientation; 
  private Scoring scoring;
  private Intake intake;

  public CubeandBaseLogic(Orientation orientation, Scoring scoring, Intake intake) {
    this.orientation = orientation;
    this.scoring = scoring;
    this.intake = intake;
    
    addRequirements(orientation);
    addRequirements(intake);
    addRequirements(scoring);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   if (orientation.isConeMode()) {
    
     //enum code here
   }

   else if (!orientation.isConeMode()) {
     //enum code here
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
