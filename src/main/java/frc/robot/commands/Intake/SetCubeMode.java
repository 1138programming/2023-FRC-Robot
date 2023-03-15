// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Claw;
import static frc.robot.Constants.*;

public class SetCubeMode extends CommandBase {
  /** Creates a new SetCubeMode. */

  private Orientation orientation; 
  private Claw claw;
  private Intake intake;
  private Limelight limelight;

  public SetCubeMode(Orientation orientation, Intake intake, Claw claw, Limelight limelight) {
    this.orientation = orientation;
    this.intake = intake;
    this.claw = claw;
    this.limelight = limelight;

    addRequirements(orientation);
    addRequirements(intake);
    addRequirements(claw);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setCubeMode();
    orientation.setCubeMode();
    claw.setCubeMode();
    limelight.setPipeline(KAprilTagPipeline);
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
