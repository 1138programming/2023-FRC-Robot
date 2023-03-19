// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.IntakeThenLift.LiftReadySetpoint;
import frc.robot.commands.Scoring.Claw.OpenClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreThenFlipBack extends SequentialCommandGroup {
  /** Creates a new ScoreThenFlipBack. */
  public ScoreThenFlipBack(Claw claw, Lift lift, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OpenClaw(claw),
      new WaitCommand(0.5),
      new InnerLiftFlipIn(lift),
      new LiftReadySetpoint(intake, lift, claw)
    );
  }
}
