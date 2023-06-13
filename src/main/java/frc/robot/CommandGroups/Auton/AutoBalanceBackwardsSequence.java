// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.AutoBalance;
import frc.robot.commands.Base.DriveForward;
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.DriveUntilFloor;
import frc.robot.commands.Base.DriveUntilStation;
import frc.robot.commands.Base.DriveUntilStationBackwards;
import frc.robot.commands.Base.ResetAllButGyro;
import frc.robot.commands.Base.ResetEncodersTeleop;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ResetOdometry;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceBackwardsSequence extends SequentialCommandGroup {
  /** Creates a new AutoBalanceAuton. */
  public AutoBalanceBackwardsSequence(Base base, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ResetOdometry(base),
      // new ParallelRaceGroup(
      //   new WaitCommand(2),
      //   new DriveToPose(base, new Pose2d(0, 0, Rotation2d.fromDegrees(180)))
      // ),
      // new ParallelRaceGroup(
      //   new WaitCommand(3),
      //   new DriveUntilStationBackwards(base)
      // ),
      // new ResetOdometry(base),
      // new ParallelRaceGroup(
      //   new WaitCommand(1),
      //   new DriveToPose(base, new Pose2d(-0.5, 0, new Rotation2d()))
      // ),
      // new AutoBalance(base)
      new ParallelRaceGroup(
        new IntakeUpAuton(intake, 3),
        new WaitCommand(3),
        new DriveUntilStation(base)
      ),
      new ParallelRaceGroup(
        new IntakeUpAuton(intake, 4),
        new WaitCommand(4),
        new DriveUntilFloor(base)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.55),
        new DriveForward(base, KTimedDriveSpeed)
      ),
      new ResetAllButGyro(base),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(0.03),
      //   new ResetAllButGyro(base)
      // ),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new DriveToPose(
        base, 
        new Pose2d(0, 0, Rotation2d.fromDegrees(45))
      )
      ),
      
      new WaitCommand(1),
      new ParallelRaceGroup(
        new IntakeUpAuton(intake, 3),
        new WaitCommand(3),
        new DriveUntilStationBackwards(base)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.03),
        new ResetEncodersTeleop(base)
      ),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new IntakeUpAuton(intake, 1),
        new DriveToPose(base, new Pose2d(-0.5, 0, new Rotation2d()))
      ),
      new AutoBalance(base)
    );
  }
}
