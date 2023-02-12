// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;


public class IntakeSwivelTop extends CommandBase {
 /** Creates a new intakeSpaghettiSet. */
  private Intake intake;
  
  public IntakeSwivelTop (Intake intake, double encoderTarget ) {
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
    intake.swivelSpinToPos(KIntakeSwivelTopPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intake.getIntakeEncoder() <= KIntakeSwivelTopPos + KIntakeSwiveTopOffset && intake.getIntakeEncoder() <= KIntakeSwivelTopPos - KIntakeSwiveTopOffset) {
       return true;
      }
      else {
    return false;
  }
  }
}
