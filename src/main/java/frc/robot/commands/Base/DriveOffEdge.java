// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import frc.robot.subsystems.Base;
import frc.robot.subsystems.Endgame;

import static frc.robot.Constants.*;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveOffEdge extends CommandBase {
  /** Creates a new DriveOffEdge. */
  Base base;
  Endgame endgame;
  public DriveOffEdge(Base base, Endgame endgame) {
    this.base = base;
    this.endgame = endgame;
    addRequirements(base, endgame);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    base.drive(0, -KEndgameDriveSpeed, 0, false, KPhysicalMaxDriveSpeedMPS);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endgame.getFrontIR() || endgame.getBackIR()) {
    return true;
    }
    return false;
  }
}
