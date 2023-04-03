// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import static frc.robot.Constants.*;

public class DriveToPose extends CommandBase {
  private final Base base;
  private Pose2d currentPose;
  private Pose2d targetPose;

  private double headingOffset;
  private double yOffset;
  private double xOffset;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private double rotP = 4.5;
  private double rotI = 0;
  private double rotD = 0;

  private PIDController fbController;
  private PIDController lrController;
  private PIDController rotController;

  private SlewRateLimiter fbSpeedLimiter;
  private SlewRateLimiter lrSpeedLimiter;

  /** Creates a new DriveToPose. */
  public DriveToPose(Base base, Pose2d targetPose) {
    this.base = base;
    this.targetPose = targetPose;

    currentPose = base.getPose();

    fbController = new PIDController(0.85, 0, 0);
    lrController = new PIDController(0.85, 0, 0);
    // fbController = new PIDController(0.50, 0.001, 0);
    // lrController = new PIDController(0.50, 0.001, 0);
    rotController = new PIDController(rotP, rotI, rotD);

    fbSpeedLimiter = new SlewRateLimiter(2);
    lrSpeedLimiter = new SlewRateLimiter(2);

    addRequirements(base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.resetAllRelEncoders();  
    // base.resetOdometry();
    // base.resetGyro();
  }

  @Override
  public void execute() {

    // rotController.setPID(SmartDashboard.getNumber("rotP", 0), SmartDashboard.getNumber("rotI", 0), SmartDashboard.getNumber("rotD", 0));

    currentPose = base.getPose();
    xOffset = targetPose.getX() - currentPose.getX();
    yOffset = targetPose.getY() - currentPose.getY();
    headingOffset = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
    SmartDashboard.putNumber("xOffset", xOffset);
    SmartDashboard.putNumber("yOffset", yOffset);

    xSpeed = fbController.calculate(currentPose.getX(), targetPose.getX());
    ySpeed = lrController.calculate(currentPose.getY(), targetPose.getY());

    xSpeed = fbSpeedLimiter.calculate(xSpeed);
    ySpeed = lrSpeedLimiter.calculate(ySpeed);
    rotSpeed = rotController.calculate(currentPose.getRotation().getDegrees()/360, targetPose.getRotation().getDegrees()/360);
    
    base.drive(xSpeed, ySpeed, rotSpeed, true, KPhysicalMaxDriveSpeedMPS);
  }

  @Override
  public void end(boolean interrupted) {
    base.drive(0, 0, 0, true, KPhysicalMaxDriveSpeedMPS);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(xOffset) < 0.06 && Math.abs(yOffset) < 0.06 && Math.abs(headingOffset) < 0.1;
  }
}