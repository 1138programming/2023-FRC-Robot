// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Base.ResetEncodersTeleop;
import frc.robot.commands.Intake.SetConeMode;
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
public class ConeHighBalance extends SequentialCommandGroup {
  /** Creates a new ConeHighLeave. */
  public ConeHighBalance(Base base, Intake intake, Limelight limelight, Lift lift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetConeMode(intake, limelight, lift),
      new ResetEncodersTeleop(base),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new MoveLiftToHighPos(lift)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new OuttakeRollers(lift)
      ),
      new ParallelRaceGroup(
        new WaitCommand(3),
        new MoveLiftToReadyPos(lift)
      ),
      new AutoBalanceSequence(base)
    );
  }
}
