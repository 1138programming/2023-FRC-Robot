// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.AutoBalance;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.DriveUntilStation;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.subsystems.Base;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceSequence extends SequentialCommandGroup {
  /** Creates a new AutoBalanceAuton. */
  public AutoBalanceSequence(Base base) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveUntilStation(base),
      new ResetOdometry(base),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new DriveToPose(base, new Pose2d(0.55, 0, new Rotation2d()))
      ),
      new AutoBalance(base)
    );
  }
}