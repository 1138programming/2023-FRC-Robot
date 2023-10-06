// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base.Drives;

import frc.robot.Robot;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithJoysticks extends CommandBase {

  private final Base base;

  private double fbSpeed; //Speed of the robot in the x direction (forward).
  private double lrSpeed; //Speed of the robot in the Y direction (sideways).
  private double rot;

  private double kRotationP = 0.005;
  private double kRotationI = 0;
  private double kRotationD = 0;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Base base) {
    this.base = base;

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fbSpeed = Robot.m_robotContainer.getLogiLeftYAxis();
    lrSpeed = Robot.m_robotContainer.getLogiLeftXAxis();
    
    rot = Robot.m_robotContainer.getLogiRightXAxis();

    base.drive(fbSpeed, lrSpeed, rot, true, KPhysicalMaxDriveSpeedMPS * base.getDriveSpeedFactor());
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