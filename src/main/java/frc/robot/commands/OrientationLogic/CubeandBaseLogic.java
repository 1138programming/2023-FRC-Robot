// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OrientationLogic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Scoring;
import static frc.robot.Constants.*;

public class CubeandBaseLogic extends CommandBase {
  /** Creates a new OrientationMove. */

  private Orientation orientation; 
  private Scoring scoring;

  public CubeandBaseLogic(Orientation orientation) {
    this.orientation = orientation;
    addRequirements(orientation);
  }

  public CubeandBaseLogic(Scoring scoring){
    this.scoring = scoring;
    addRequirements(scoring);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    orientation.moveOrientationLeftandRightMotors();
    orientation.moveOrientationMotorExtension(KMotorExtensionSpeed);

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
