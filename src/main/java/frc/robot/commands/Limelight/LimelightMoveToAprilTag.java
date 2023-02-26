// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import static frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class LimelightMoveToAprilTag extends CommandBase {
  private PIDController rotControl;
  private PIDController BaseController; 
  private Base base;
  private Limelight limelight;
  private double margin;
  private double Xdistance;
  private double Ydistance;

  /** Creates a new LimelightMoveToAprilTag. */
  public LimelightMoveToAprilTag(Base base, Limelight limelight) {
    rotControl = new PIDController(KLimelightRotateP, KLimelightRotateI, KLimelightRotateD); 
    BaseController = new PIDController(KLimelightMoveP, KLimelightMoveI,KLimelightMoveD);
    this.base = base;
    this.limelight = limelight;
     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(base, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      margin = limelight.getAngle();
      
      double rotSpeed = rotControl.calculate(margin, 0);

      Xdistance = -(limelight.getXOffset() - KXCoordinateoOfTag); 
      Ydistance = -(limelight.getYOffset() - KYCoordinateoOfTag);

      
      
      double XSpeed = BaseController.calculate(Xdistance, 0);
      double YSpeed = BaseController.calculate(Ydistance, 0);

      base.drive(XSpeed, YSpeed, 0, false, KPhysicalMaxDriveSpeedMPS);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (Xdistance < KDistanceMoveOffset && Ydistance < KDistanceMoveOffset)            
    {
      return false;
    }
    else {
     return false;
    }
  }
}
