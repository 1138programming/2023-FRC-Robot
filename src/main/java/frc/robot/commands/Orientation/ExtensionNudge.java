// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Orientation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Orientation;
import static frc.robot.Constants.*;

public class ExtensionNudge extends CommandBase {
  /** Creates a new OrientationMove. */

  private Orientation orientation; 
  private Timer timer = new Timer();

  public ExtensionNudge(Orientation orientation) {
    this.orientation = orientation;
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      orientation.moveOrientationMotorExtension(KExtensionMotorSpeed);
      // orientation.moveOrientationLeftandRightMotors();
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    orientation.moveOrientationMotorExtension(0);
    //orientation.moveOrientationLeftandRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(KMotorExtensionTime)){
      return true;
  }
  return false;
  }
}
