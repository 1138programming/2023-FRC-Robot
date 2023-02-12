// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.Orientation;

public class ConeAndCubeMode extends CommandBase {
  /** Creates a new ConeAndCubeMode. */
  Intake intake;
  Orientation orientation;

  public ConeAndCubeMode(Intake intake, Orientation orientation) {
    this.intake = intake;
    this.orientation = orientation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (intake.getMode()) {
      intake.setCubeMode();
    }
    else if (!intake.getMode()) {
      intake.setConeMode();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   

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
