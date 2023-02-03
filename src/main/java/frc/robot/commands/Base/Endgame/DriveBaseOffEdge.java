// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base.Endgame;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.KPhysicalMaxDriveSpeedMPS;



public class DriveBaseOffEdge extends CommandBase {
  Endgame endgame;
  Base base;
  /** Creates a new DriveBaseOffEdge. */
  public DriveBaseOffEdge(Endgame endgame, Base base) {
    this.base = base;
    this.endgame = endgame;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endgame, base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.drive(0.1, 0, 0, isFinished(), KPhysicalMaxDriveSpeedMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endgame.isLimitSwitchPressed()) {
      return true;
    }
    return false;
  }
}
