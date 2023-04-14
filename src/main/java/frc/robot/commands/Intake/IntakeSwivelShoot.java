// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class IntakeSwivelShoot extends CommandBase {
  /** Creates a new IntakeSwivelBottum. */
  private Intake intake;
  private double intakeSetpoint = KIntakeSwivelShootPos;

  public IntakeSwivelShoot(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.swivelSpinToPos(intakeSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(intake.getSwivelEncoder() - intakeSetpoint) < KIntakeSwivelOffset;
  }
}