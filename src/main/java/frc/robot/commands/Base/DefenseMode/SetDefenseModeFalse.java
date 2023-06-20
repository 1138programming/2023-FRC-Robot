// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Base.DefenseMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;

public class SetDefenseModeFalse extends CommandBase {
  private Base base;
  private Intake intake;
  /** Creates a new SetDefenseModeFalse. */
  public SetDefenseModeFalse(Base base, Intake intake) {
    this.base = base;
    this.intake = intake;
    addRequirements(base, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    base.setDefenseMode(false);
    if (intake.getObjectMode() == KConeMode) {
      intake.setConeMode();
    }
    else {
      intake.setCubeMode();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.l  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
