// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoring;

public class flipToPos extends CommandBase {
  /** Creates a new flipToPos. */
  private Scoring scoring;
  private double scoringPos;

  public flipToPos(Scoring scoring, double scoringPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoring = scoring;
    this.scoringPos = scoringPos;
    addRequirements(scoring);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scoring.flipToPos(scoringPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (scoring.getFlipperPos() <  scoringPos+0.5 && scoring.getFlipperPos() > scoringPos-0.5) {
      return true;
    }
    else {
      return false;
    }
  }
}
