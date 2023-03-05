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
  private double xCoordinateAprilTag;
  private double yCoordinateAprilTag;

  /** Creates a new LimelightMoveToAprilTag. */
  public LimelightMoveToAprilTag(Base base, Limelight limelight) {
    rotControl = new PIDController(KLimelightRotateP, KLimelightRotateI, KLimelightRotateD); 
    BaseController = new PIDController(KTagLimelightMoveP, KTagLimelightMoveI,KTagLimelightMoveD);
    this.base = base;
    this.limelight = limelight;
     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(base, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int id = (int) limelight.getTID() - 1;
    if (limelight.getTargetFound() && id < 8 && id >= 0) {
      xCoordinateAprilTag = KXCoordinateOfTag[id];
      yCoordinateAprilTag = KYCoordinateOfTag[id];
    }

    margin = limelight.getXAngle();
    
    double rotSpeed = rotControl.calculate(margin, 0);

    Xdistance = Math.abs(limelight.getTagXOffset() - xCoordinateAprilTag); 
    Ydistance = -(limelight.getTagYOffset() - yCoordinateAprilTag);
    
    double XSpeed = BaseController.calculate(Ydistance, 0);  // swapped for testing
    double YSpeed = BaseController.calculate(Xdistance, 0);
    // double XSpeed = BaseController.calculate(Xdistance, 0);
    // double YSpeed = BaseController.calculate(Ydistance, 0);

    SmartDashboard.putNumber("xdist", Xdistance);
    SmartDashboard.putNumber("ydist", Ydistance);
    
    if (id >= 0 && id < 8)
      base.drive(XSpeed, 0, rotSpeed, false, KPhysicalMaxDriveSpeedMPS);
    
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
