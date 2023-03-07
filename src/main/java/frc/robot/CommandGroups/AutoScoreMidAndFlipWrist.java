// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Scoring;
import frc.robot.commands.Scoring.moveWristServo;
import frc.robot.commands.Scoring.flipToPos;
import frc.robot.commands.Scoring.moveLiftToPos;
import static frc.robot.Constants.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreMidAndFlipWrist extends SequentialCommandGroup {
  /** Creates a new AutoScoreHighAndFlipWrist. */
  Scoring scoring;
  public AutoScoreMidAndFlipWrist(Scoring scoring) {
    this.scoring = scoring;
    
    addCommands(
      new ParallelCommandGroup( 
        new moveLiftToPos(scoring, KLiftMediumPos),
        new flipToPos(scoring, KScoringFlipPos)
      ),
      new moveWristServo(scoring)
    );
  }
}
