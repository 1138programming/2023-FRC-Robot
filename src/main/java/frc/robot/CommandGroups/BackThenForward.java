// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Base.*;
import frc.robot.commands.Intake.IntakeSpinReverse;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackThenForward extends SequentialCommandGroup {
  /** Creates a new BackThenForward. */
  public BackThenForward(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEncoders(base),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5), 
        //new IntakeSpinReverse(intake),
        new DriveForward(base)
      ),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(1),
      //   new IntakeSpinReverse(intake)
      // ),
      new ParallelDeadlineGroup(
        new WaitCommand(3), 
        new DriveBackward(base)
      )
    );
  }
}
