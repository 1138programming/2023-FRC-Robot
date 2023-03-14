// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class DriveForward extends CommandBase {
  private Base base;
  /** Creates a new DriveForward. */
  public DriveForward(Base base) {
    this.base = base;
    addRequirements(base);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(0.5, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
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
