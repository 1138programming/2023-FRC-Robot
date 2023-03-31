// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;
import static frc.robot.Constants.*;

public class InnerLiftIn extends CommandBase {
  private Lift lift;
  private double setpoint = KInnerLiftInPos;
  /** Creates a new InnerLiftOut. */
  public InnerLiftIn(Lift lift) {
    this.lift = lift;
    addRequirements(lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lift.setInnerLiftPos(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.moveInnerLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(setpoint - lift.getInnerLiftPos()) < KInnerLiftDeadzone;
  }
}
