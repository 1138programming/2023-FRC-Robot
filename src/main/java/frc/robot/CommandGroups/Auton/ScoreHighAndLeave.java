// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.IntakeThenLift.LiftHighSetpoint;
import frc.robot.CommandGroups.IntakeThenLift.LiftReadySetpoint;
import frc.robot.commands.Intake.IntakeMoveSwivelDown;
import frc.robot.commands.Scoring.Claw.CloseClaw;
import frc.robot.commands.Scoring.Claw.OpenClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Base;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighAndLeave extends SequentialCommandGroup {
  /** Creates a new ScoreHighAndLeave. */
  public ScoreHighAndLeave(Lift lift, Claw claw, Base base, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new CloseClaw(claw)
      ),
      new ParallelRaceGroup(
        new WaitCommand(4),
        new LiftHighSetpoint(lift, claw)
      ),
      new WaitCommand(1),
      new ParallelDeadlineGroup(
        new WaitCommand(2), 
        new OpenClaw(claw)
      ),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(0.3), 
      //   new IntakeMoveSwivelDown(intake)
      // ),
      new ParallelRaceGroup(
        new WaitCommand(4),
        new LiftReadySetpoint(lift, claw)
      ),
      new TimedDriveForward(base)
    );
  }
}
