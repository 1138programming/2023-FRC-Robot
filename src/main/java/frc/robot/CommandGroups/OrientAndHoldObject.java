// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Orientation.MoveExtensionToInPosition;
import frc.robot.commands.Scoring.Claw.CloseClaw;
import frc.robot.commands.Scoring.Lift.MoveLiftToLowPos;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrientAndHoldObject extends SequentialCommandGroup {
  /** Creates a new OrientAndHoldObject. */
  Lift lift;
  Claw claw;
  Orientation orientation;
  public OrientAndHoldObject(Lift lift, Claw claw, Orientation orientation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.lift = lift;
    this.claw = claw;
    this.orientation = orientation;



    addCommands(
      new ParallelCommandGroup(  
        new MoveExtensionToInPosition(orientation),
        new MoveLiftToLowPos(lift)
      ),
      new CloseClaw(claw) 

      
    );
  }
}
