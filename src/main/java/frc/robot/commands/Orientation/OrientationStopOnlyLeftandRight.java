// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Orientation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Orientation;
import static frc.robot.Constants.*;

public class OrientationStopOnlyLeftandRight extends CommandBase {
  /** Creates a new OrientationMove. */

  private Orientation orientation; 

  public OrientationStopOnlyLeftandRight(Orientation orientation) {
    this.orientation = orientation;
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    orientation.stopOrientationLeftandRightMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // orientation.stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

