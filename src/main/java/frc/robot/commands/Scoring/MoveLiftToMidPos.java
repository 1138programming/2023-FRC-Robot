// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoring;
import static frc.robot.Constants.*;

public class MoveLiftToMidPos extends CommandBase {
  Scoring scoring;
  public MoveLiftToMidPos(Scoring scoring) {
    this.scoring = scoring;
    addRequirements(scoring);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    scoring.moveLift(KLiftMediumPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (KLiftReadyPos+? >=  scoring.getLiftPos() || KLiftReadyPos-? <=  scoring.getLiftPos()){
    //   return true;
    // }
    return false;
  }
}
