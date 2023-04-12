
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Scoring.Flipper.FlipperRollerSpin;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Lift;
import static frc.robot.Constants.*;

public class FlipperToScoringSetPos extends CommandBase {
  private Lift lift;
  private Flipper flipper;
  private double setPosition = KFlipperOutPos;
  private double speed;
  /** Creates a new FlipperToReadyPos. */
  public FlipperToScoringSetPos(Lift lift) {
    this.lift = lift;
    this.flipper = flipper;
    this.speed = speed;
    addRequirements(lift, flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.flipToPos(KFlipperSwivelCanCoderOffShelfVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.moveFlipperSwivel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(setPosition - lift.getFlipperPos()) < KFlipperDeadzone;
  }
}

