// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring.Flipper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flipper;
import static frc.robot.Constants.*;

public class FlipperRollerSpin extends CommandBase {
  /** Creates a new ClawRollersSpin. */
  Flipper flipper;
  Double speed;
  public FlipperRollerSpin(Flipper claw, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flipper = claw;
    this.speed = speed;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flipper.spinRoller(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
