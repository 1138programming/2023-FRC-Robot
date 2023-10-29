// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.AutonSequences;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.Auton.AutonUtility.AutoBalanceSequence;
import frc.robot.CommandGroups.Auton.AutonUtility.DriveToPose;
import frc.robot.commands.Base.Resets.ResetEncodersTeleop;
import frc.robot.commands.Base.Resets.ResetGyroOffset;
import frc.robot.commands.Intake.IntakeSpaghettiShoot;
import frc.robot.commands.Intake.IntakeSwivelShoot;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.commands.Intake.SetCubeMode;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeShootBalance extends SequentialCommandGroup {
  /** Creates a new ConeHighLeave. */
  public CubeShootBalance(Base base, Intake intake, Limelight limelight, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetCubeMode(intake, limelight, lift),
      new ResetEncodersTeleop(base),
      new ResetGyroOffset(base, KGyroOffset),
      new ParallelRaceGroup(
        new WaitCommand(1.5),
        new IntakeSwivelShoot(intake)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new IntakeSpaghettiShoot(intake)
      ),
      new ParallelRaceGroup(
        new WaitCommand(1.5),
        new IntakeSwivelTop(intake)
      ),
      new ResetEncodersTeleop(base),
      new ResetGyroOffset(base, KGyroOffset),
      new DriveToPose(base, new Pose2d(0.08, 0, Rotation2d.fromDegrees(0))),
      new AutoBalanceSequence(base)
    );
  }
}
