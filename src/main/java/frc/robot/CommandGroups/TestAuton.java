// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.FollowPath;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

import com.pathplanner.lib.PathPlannerTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuton extends SequentialCommandGroup {
  private PathPlannerTrajectory pathPlannerTrajectory;

  /** Creates a new TestAuton. */
  public TestAuton(Base base) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(base),
      base.followTrajectoryCommand(KPath1, true)
    );
  }
}
