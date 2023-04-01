// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Orientation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Orientation;

public class OrientationCheckCube extends CommandBase {
  private Orientation orientation;
  /** Creates a new OrientationCheckDoor. */
  public OrientationCheckCube(Orientation orientation) {
    this.orientation = orientation;
    addRequirements(orientation);
  }

  @Override
  public void execute() {
    orientation.moveOrientationLeftandRightMotors(0.5);
  }

  @Override
  public boolean isFinished() {
    return (orientation.getDoorRange() > 5 && orientation.getDoorRange() < 8);
  }
}
