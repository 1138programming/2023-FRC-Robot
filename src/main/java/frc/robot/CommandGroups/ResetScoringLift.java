// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Scoring;
import frc.robot.commands.Scoring.CloseClaw;
import frc.robot.commands.Scoring.MoveLiftToReadyPos;
import frc.robot.commands.Scoring.OpenClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetScoringLift extends SequentialCommandGroup {
  /** Creates a new ResetScoringLift. */
  Scoring scoring; 
  public ResetScoringLift(Scoring scoring) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.scoring = scoring;
    addCommands(
      new CloseClaw(scoring),
      new MoveLiftToReadyPos(scoring),
      new OpenClaw(scoring)
    );
  }
}
