// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import static frc.robot.Constants.*;

public class SetConeMode extends CommandBase {
  /** Creates a new SetConeMode. */

  private Intake intake;
  private Lift lift;
  private Limelight limelight;

  public SetConeMode(Intake intake, Limelight limelight, Lift lift) {
    this.intake = intake;
    this.limelight = limelight;
    this.lift = lift;

    addRequirements(intake, limelight, lift);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setConeMode();
    lift.setConeMode();
    limelight.setPipeline(KReflectiveTapePipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
