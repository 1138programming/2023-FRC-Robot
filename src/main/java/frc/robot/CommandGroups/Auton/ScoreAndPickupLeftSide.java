// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.DriveForward;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndPickupLeftSide extends SequentialCommandGroup {
  /** Creates a new ScoreAndPickup. */
  public ScoreAndPickupLeftSide(Base base, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ParallelDeadlineGroup(
      //   new WaitCommand(0.5), 
      //   new DriveForward(base)
      // )
      new IntakeSpit(intake),
      new ParallelDeadlineGroup(
        new WaitCommand(10), 
        base.followTrajectoryCommand(KPickUpLeftSide, true), 
        new IntakeSpin(intake)
      )
      // base.followTrajectoryCommand(KPickUpRightSide, true)
      
    );
  }
}
