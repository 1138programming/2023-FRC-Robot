// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.OldAuton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.Auton.AutoBalanceSequence;
import frc.robot.commands.Base.ResetEncoders;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftSideGrabScoreBalance extends SequentialCommandGroup {
  /** Creates a new PickUpLeftSide. */
  public LeftSideGrabScoreBalance(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEncoders(base),
      base.followTrajectoryCommand(KPickUpLeftSide, true),
      base.followTrajectoryCommand(KLeftSideCubeToStation, false),
      base.followTrajectoryCommand(KLeftSideCubeToBalance, false),
      new AutoBalanceSequence(base)
    );
  }
}
