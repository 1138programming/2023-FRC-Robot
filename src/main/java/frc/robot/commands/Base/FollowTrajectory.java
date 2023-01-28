// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class FollowTrajectory extends CommandBase {
  private Base base;

  private PIDController xController;
  private PIDController yController;
  private ProfiledPIDController rotController;
  private TrapezoidProfile.Constraints constraints;
  private HolonomicDriveController HDC;

  
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Base base) {
    this.base = base;
    
    xController = new PIDController(KXControllerP, KXControllerI, KXControllerD);
    yController = new PIDController(KYControllerP, KYControllerI, KYControllerD);
    constraints = new TrapezoidProfile.Constraints(KRotMaxVelocity, KRotMaxAcceleration);
    rotController = new ProfiledPIDController(KRotControllerP, KRotControllerI, KRotControllerD, constraints);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
