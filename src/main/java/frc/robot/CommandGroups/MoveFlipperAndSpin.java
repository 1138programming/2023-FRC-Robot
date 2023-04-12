// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Scoring.Flipper.FlipperRollerSpin;
import frc.robot.commands.Scoring.Lift.FlipperToScoringSetPos;
import frc.robot.commands.Scoring.Lift.MoveFlipperRoller;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Lift;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveFlipperAndSpin extends SequentialCommandGroup {
  /** Creates a new MoveFlipperAndScore. */
  public MoveFlipperAndSpin(Flipper flipper, Lift lift, double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
          new FlipperToScoringSetPos(lift),
            new ParallelCommandGroup(
              new WaitCommand(2),
              new FlipperRollerSpin(flipper, speed)
              )
            )
          );
  }
}
