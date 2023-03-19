// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.IntakeThenLift;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.InnerLiftFlipOut;
import frc.robot.commands.Intake.IntakeMoveSwivelDown;
import frc.robot.commands.Scoring.Lift.MoveLiftToHighPos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftHighSetpoint extends SequentialCommandGroup {
  /** Creates a new IntakeThenLiftUp. */
  public LiftHighSetpoint(Intake intake, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new IntakeMoveSwivelDown(intake),
        new WaitCommand(KIntakeThenLiftTime)
      ),
      new MoveLiftToHighPos(lift),
      new InnerLiftFlipOut(lift)
    );
  }
}
