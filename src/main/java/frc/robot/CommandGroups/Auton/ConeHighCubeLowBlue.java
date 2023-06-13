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
import frc.robot.commands.Base.DriveToPose;
import frc.robot.commands.Base.ResetEncodersTeleop;
import frc.robot.commands.Intake.IntakeBottomNoCollect;
import frc.robot.commands.Intake.IntakeShootOut;
import frc.robot.commands.Intake.IntakeSwivelBottom;
import frc.robot.commands.Intake.IntakeSwivelShoot;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.commands.Intake.SetConeMode;
import frc.robot.commands.Intake.SetCubeMode;
import frc.robot.commands.Scoring.Lift.MoveLiftToHighPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToReadyPos;
import frc.robot.commands.Scoring.Lift.OuttakeRollers;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeHighCubeLowBlue extends SequentialCommandGroup {
  /** Creates a new ConeHighLeave. */
  public ConeHighCubeLowBlue(Base base, Intake intake, Limelight limelight, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetConeMode(intake, limelight, lift),
      new ResetEncodersTeleop(base),
      new ConeHigh(base, intake, limelight, lift),
      new SetCubeMode(intake, limelight, lift),
      new ParallelRaceGroup(
        new DriveToPose(base, new Pose2d(4.3, -0.45, new Rotation2d())),
        new IntakeSwivelBottom(intake),
        new WaitCommand(4)
      ),
      new ParallelRaceGroup(
        new DriveToPose(base, new Pose2d(0.5, -0.2, Rotation2d.fromDegrees(145))),
        new IntakeSwivelTop(intake),
        new WaitCommand(3)
      ),
      new ParallelRaceGroup(
        new DriveToPose(base, new Pose2d(0.2, -0.2, Rotation2d.fromDegrees(180))),
        new IntakeBottomNoCollect(intake),
        new WaitCommand(2)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1), 
        new IntakeShootOut(intake)
      ),
      new ParallelRaceGroup(
        new IntakeSwivelTop(intake),
        new WaitCommand(2)
      )
    );
  }
}