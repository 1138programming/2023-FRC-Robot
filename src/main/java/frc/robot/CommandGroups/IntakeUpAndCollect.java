// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Orientation.OrientationCheckCube;
import frc.robot.commands.Scoring.Claw.CloseClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orientation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeUpAndCollect extends SequentialCommandGroup {
  /** Creates a new IntakeUpAndCollect. */
  public IntakeUpAndCollect(Orientation orientation, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OrientationCheckCube(orientation),
      new CloseClaw(claw)

    );
  }
}
