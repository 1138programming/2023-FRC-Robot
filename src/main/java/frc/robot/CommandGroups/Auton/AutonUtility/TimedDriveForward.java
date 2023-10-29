// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.AutonUtility;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.Resets.ResetEncoders;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedDriveForward extends SequentialCommandGroup {
  /** Creates a new TimedDriveForward. */
  public TimedDriveForward(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEncoders(base),
      new ParallelDeadlineGroup(
        new WaitCommand(3.5),
        new DriveForward(base, KTimedDriveSpeed)
      )
    );
  }
}
