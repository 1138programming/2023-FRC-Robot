// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.OldAuton;

import frc.robot.commands.Base.AutoBalance;
import frc.robot.subsystems.Base;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceAuton extends SequentialCommandGroup {
  // private Base base;
  /** Creates a new AutoBalanceAuton. */
  public AutoBalanceAuton(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoBalance(base)
    );
  }
}
