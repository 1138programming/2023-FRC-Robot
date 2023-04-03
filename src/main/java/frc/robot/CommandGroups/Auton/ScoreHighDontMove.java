// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.Intake.IntakeMoveSwivelDown;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Base;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighDontMove extends SequentialCommandGroup {
  /** Creates a new ScoreHighAndLeave. */
  public ScoreHighDontMove(Lift lift, Claw claw, Base base, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(1)
      ),
      new ParallelRaceGroup(
        new WaitCommand(4)
      ),
      new WaitCommand(1),
      new ParallelDeadlineGroup(
        new WaitCommand(2)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.3), 
        new IntakeMoveSwivelDown(intake)
      ),
      new ParallelRaceGroup(
        new WaitCommand(4)
      )
    );
  }
}
