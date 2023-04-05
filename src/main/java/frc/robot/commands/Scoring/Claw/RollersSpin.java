package frc.robot.commands.Scoring.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import static frc.robot.Constants.*;

public class RollersSpin extends CommandBase {
    
   private Claw claw;

   public RollersSpin (Claw claw) {
    this.claw = claw;
    addRequirements(claw);
   }

    @Override
    public void initialize() {

    }

    @Override
    public void execute () {
        claw.spinRoller(KClawMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished () {
        return false;
    }

}





