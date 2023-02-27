// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class ToggleLimelightPipeline extends CommandBase {
  private Limelight limelight;
  /** Creates a new ToggleLimelightPipeline. */
  public ToggleLimelightPipeline(Limelight limelight) {
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelight.getPipeline() == 0) {
      limelight.setPipeline(1);
    } else {
      limelight.setPipeline(0);
    }
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
