// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import frc.robot.subsystems.Base;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Base.DriveWithJoysticks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefenseModeDrive extends ParallelCommandGroup {
  /** Creates a new DefenseModeDrive. */
  public DefenseModeDrive(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveWithJoysticks(base)

    );
  }
}
