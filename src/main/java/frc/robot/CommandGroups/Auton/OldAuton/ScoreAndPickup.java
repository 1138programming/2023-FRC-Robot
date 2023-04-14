// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.OldAuton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auton.DriveToPose;
import frc.robot.commands.Base.BaseRotate;
import frc.robot.commands.Base.DriveBackward;
import frc.robot.commands.Base.DriveForward;
import frc.robot.commands.Base.ResetEncoders;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndPickup extends SequentialCommandGroup {
  /** Creates a new ScoreAndPickup. */
  public ScoreAndPickup(Base base, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEncoders(base),
      new IntakeSpit(intake),
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new BaseRotate(base)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(4),
        new IntakeSpin(intake),
        new DriveBackward(base)
      )
    );
  }
}
