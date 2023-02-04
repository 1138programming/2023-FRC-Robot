// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class FollowPath extends CommandBase {
  private Base base;
  private PathPlannerTrajectory pathPlannerTrajectory;
  private boolean isFirstPath;

  /** Creates a new FollowPath. */
  public FollowPath(Base base, String pathName, double maxVelocity, double maxAcceleration, boolean isFirstPath) {
    this.base = base;
    this.isFirstPath = isFirstPath;
    pathPlannerTrajectory = base.getPathPlannerTrajectory(pathName, maxVelocity, maxAcceleration);
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    base.followTrajectoryCommand(pathPlannerTrajectory, isFirstPath);
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
