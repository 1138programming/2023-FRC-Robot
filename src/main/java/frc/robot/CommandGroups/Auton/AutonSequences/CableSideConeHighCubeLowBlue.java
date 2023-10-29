// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton.AutonSequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandGroups.Auton.AutonUtility.ConeHigh;
import frc.robot.CommandGroups.Auton.AutonUtility.DriveToPose;
import frc.robot.CommandGroups.Auton.AutonUtility.IntakeUpAuton;
import frc.robot.commands.Base.Resets.ResetEncodersTeleop;
import frc.robot.commands.Intake.IntakeSwivelBottom;
import frc.robot.commands.Intake.IntakeSpaghettiShoot;
import frc.robot.commands.Intake.IntakeSwivelAndCollect;
import frc.robot.commands.Intake.IntakeSwivelShoot;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.commands.Intake.SetConeMode;
import frc.robot.commands.Intake.SetCubeMode;
import frc.robot.commands.Lift.MoveLiftToHighPos;
import frc.robot.commands.Lift.MoveLiftToReadyPos;
import frc.robot.commands.Lift.FlipperOuttake;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CableSideConeHighCubeLowBlue extends SequentialCommandGroup {
  /** Creates a new ConeHighLeave. */
  public CableSideConeHighCubeLowBlue(Base base, Intake intake, Limelight limelight, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetConeMode(intake, limelight, lift),
      new ResetEncodersTeleop(base),
      new ConeHigh(base, intake, limelight, lift),
      new SetCubeMode(intake, limelight, lift),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new IntakeUpAuton(intake, 3),
        new DriveToPose(base, KCableSideCubePickupAfterCrossBlue)
      ),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new DriveToPose(base, KCableSideCubePickupAfterCrossBlue),
        new IntakeSwivelAndCollect(intake)
      ),
      new ParallelRaceGroup(
        new DriveToPose(base, KCableSideCrossCableBlue),
        new IntakeUpAuton(intake, 3),
        new WaitCommand(3)
      ),
      new ParallelRaceGroup(
        new DriveToPose(base, KCableSideShootPositionBlue),
        new IntakeSwivelBottom(intake),
        new WaitCommand(2)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1), 
        new IntakeSpaghettiShoot(intake)
      ),
      new SetConeMode(intake, limelight, lift),
      new ParallelRaceGroup(
        new IntakeSwivelTop(intake),
        new WaitCommand(2)
      )
    );
  }
}
