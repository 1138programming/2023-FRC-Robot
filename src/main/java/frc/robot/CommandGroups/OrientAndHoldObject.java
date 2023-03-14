// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Orientation.MoveExtensionToInPosition;
import frc.robot.commands.Scoring.CloseClaw;
import frc.robot.commands.Scoring.MoveLiftToLowPos;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Scoring;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrientAndHoldObject extends SequentialCommandGroup {
  /** Creates a new OrientAndHoldObject. */
  Scoring scoring;
  Orientation orientation;
  public OrientAndHoldObject(Scoring scoring, Orientation orientation) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.scoring = scoring;
    this.orientation = orientation;



    addCommands(
      new ParallelCommandGroup(  
        new MoveExtensionToInPosition(orientation),
        new MoveLiftToLowPos(scoring)
      ),
      new CloseClaw(scoring) 

      
    );
  }
}
