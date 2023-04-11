package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSpinAndSwivel extends CommandBase {
  Intake intake;

  public IntakeSpinAndSwivel(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.swivelSpinToPos(KSwivelBottomPosition);
    intake.spaghettiSpin();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return intake.getIntakeEncoder() < Math.abs() 
  }
}
