// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class IntakeSwivelBottum extends CommandBase {
  /** Creates a new IntakeSwivelBottum. */
  private Intake intake;

  public IntakeSwivelBottum(Intake intake) {
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
    intake.swivelSpinToPos(KIntakeSwivelBottumPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (intake.getIntakeEncoder() <= KIntakeSwivelBottumPos + KIntakeSwiveBottumOffset && intake.getIntakeEncoder() <= KIntakeSwivelBottumPos - KIntakeSwiveBottumOffset) {
      return true;
     }
     else {
   return false;
 }
  }
}
