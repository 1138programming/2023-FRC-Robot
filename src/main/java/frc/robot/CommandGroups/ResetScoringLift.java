// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Claw;
import frc.robot.commands.Scoring.Claw.CloseClaw;
import frc.robot.commands.Scoring.Claw.OpenClaw;
import frc.robot.commands.Scoring.Lift.FlipperToReadyPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToLowPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToReadyPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetScoringLift extends SequentialCommandGroup {
  /** Creates a new ResetScoringLift. */
  Lift lift;
  Claw claw;
  public ResetScoringLift(Lift lift, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.claw = claw;
    this.lift = lift;
    addCommands(
      new CloseClaw(claw),
      new FlipperToReadyPos(lift),
      new MoveLiftToReadyPos(lift),
      new OpenClaw(claw)
    );
  }
}