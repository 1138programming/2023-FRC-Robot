// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import java.io.SequenceInputStream;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Scoring.Lift.LiftEndConditionPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToShelf;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Lift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabOfShelforScore extends SequentialCommandGroup {
  /** Creates a new GrabOfShelf. */
  public GrabOfShelforScore(Lift lift, Flipper flipper, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
       new MoveLiftToShelf(lift),
       new SequentialCommandGroup(
        new LiftEndConditionPos(lift),
        new MoveFlipperAndSpin(flipper, lift, speed)
       )

      )
    );
  }
}
