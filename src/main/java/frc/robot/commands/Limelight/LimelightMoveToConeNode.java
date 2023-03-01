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

public class LimelightMoveToConeNode extends CommandBase {
  private PIDController rotControlTape;
  private PIDController BaseControllerTape; 
  private Base base;
  private Limelight limelight;
  private double margin;
  private double Xdistance;
  private double Ydistance;
  private double xCoordinateAprilTag;
  private double yCoordinateAprilTag;

  /** Creates a new LimelightMoveToAprilTag. */
  public LimelightMoveToConeNode(Base base, Limelight limelight) {
    rotControlTape = new PIDController(KLimelightRotateP, KLimelightRotateI, KLimelightRotateD); 
    BaseControllerTape = new PIDController(KTapeLimelightMoveP, KTapeLimelightMoveI,KTapeLimelightMoveD);
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

    margin = limelight.getXAngle();
    
    double rotSpeed = rotControlTape.calculate(margin, 0);

    Ydistance = limelight.getDistance(); 
    Xdistance = limelight.getHorizontinalDistance(base);
    
    double XSpeed = -BaseControllerTape.calculate(Xdistance, 0);  // swapped for testing
    double YSpeed = BaseControllerTape.calculate(Ydistance, 0);

    SmartDashboard.putNumber("xTapedist", Xdistance);
    SmartDashboard.putNumber("yTapedist", Ydistance);

    SmartDashboard.putNumber("Area", limelight.getArea());

    SmartDashboard.putNumber("XSpeed", XSpeed);
    SmartDashboard.putNumber("YSpeed", YSpeed);

    // base.drive(XSpeed, YSpeed, 0, false, KPhysicalMaxDriveSpeedMPS);
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
