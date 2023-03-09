// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Intake.IntakeMoveSwivelDown;
import frc.robot.commands.Intake.IntakeMoveSwivelUp;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeSpinReverse;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.SetConeMode;
import frc.robot.commands.Intake.SetCubeMode;
import frc.robot.commands.Intake.ToggleDefenseMode;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Limelight;
//Commands for the Base
import frc.robot.commands.Base.ToggleSpeed;
import frc.robot.commands.Base.ResetEncoders;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.DriveWithJoysticks;
//Commands for the Intake
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSwivelBottum;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.commands.Intake.SetConeMode;
import frc.robot.commands.Intake.SetCubeMode;
import frc.robot.commands.Intake.IntakeSpinReverse;
import frc.robot.commands.Intake.IntakeMoveSwivelUp;
import frc.robot.commands.Intake.IntakeMoveSwivelDown;
//Commands for the Orientation
import frc.robot.commands.Orientation.OrientationMoveAllForward;
import frc.robot.commands.Orientation.OrientationMoveAllReverse;
import frc.robot.commands.Orientation.OrientationSpinOnlyLeftandRightForward;
import frc.robot.commands.Orientation.OrientationSpinOnlyLeftandRightReverse;
import frc.robot.commands.Orientation.OrientationStopOnlyExtension;
import frc.robot.commands.Scoring.CloseClaw;
import frc.robot.commands.Scoring.FlipperOut;
import frc.robot.commands.Scoring.MoveWrist;
import frc.robot.commands.Scoring.OpenClaw;
import frc.robot.commands.Scoring.RotateWrist;
import frc.robot.commands.Scoring.RotateWristToReady;
import frc.robot.commands.Scoring.StopScoring;
import frc.robot.commands.Orientation.ExtensionNudge;
import frc.robot.commands.Orientation.MoveExtensionToInPosition;
import frc.robot.commands.Orientation.MoveExtensionToOutPosition;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionForward;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionReverse;

import frc.robot.commands.Limelight.LimelightMoveToAprilTag;
import frc.robot.commands.Limelight.LimelightMoveToConeNode;
import frc.robot.commands.Limelight.ToggleLimelightPipeline;

// import frc.robot.commands.Endgame.*;


import frc.robot.commands.Base.ToggleSpeed;
import frc.robot.commands.Endgame.DeployEndgame;
import frc.robot.commands.Endgame.EndgameReadyUp;
import frc.robot.commands.Endgame.EndgameToCenter;
import frc.robot.commands.Endgame.MoveEndgameShuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final Base base = new Base();
  private final Scoring scoring = new Scoring();
  private final Endgame endgame = new Endgame();
  private final Intake intake = new Intake();
  private final Orientation orientation = new Orientation();
  private final Limelight limelight = new Limelight();

  // Base 
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  private final ToggleSpeed toggleFastSpeed = new ToggleSpeed(base, KBaseDriveMaxPercent);
  private final ToggleSpeed toggleSlowSpeed = new ToggleSpeed(base, KBaseDriveLowPercent);
  private final ResetGyro resetGyro = new ResetGyro(base);

  // Intake
  private final IntakeSpin intakeSpinForward = new IntakeSpin(intake);
  private final IntakeSpinReverse intakeSpinReverse = new IntakeSpinReverse(intake);

  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final SetConeMode setConeMode = new SetConeMode(orientation, intake, scoring, limelight);
  private final SetCubeMode setCubeMode = new SetCubeMode(orientation, intake, scoring, limelight);
  private final IntakeMoveSwivelDown moveSwivelDown = new IntakeMoveSwivelDown(intake);
  private final IntakeMoveSwivelUp moveSwivelUp = new IntakeMoveSwivelUp(intake);
  private final IntakeSwivelTop intakeSwivelTop = new IntakeSwivelTop(intake);
  private final IntakeSwivelBottum intakeSwivelBottom = new IntakeSwivelBottum(intake);

  //Orientation
  private final MoveExtensionToOutPosition OrientationMoveOut = new MoveExtensionToOutPosition(orientation);
  private final MoveExtensionToInPosition OrientationMoveIn = new MoveExtensionToInPosition(orientation);
  private final ExtensionNudge OrientationNudge = new ExtensionNudge(orientation);
  private final OrientationSpinOnlyLeftandRightReverse orientationSpinInwards = new OrientationSpinOnlyLeftandRightReverse(orientation);
  private final OrientationSpinOnlyLeftandRightForward orientationSpinOutwards = new OrientationSpinOnlyLeftandRightForward(orientation);

  // Endgame
  private final MoveEndgameShuffleboard moveEndgameShuffleboard = new MoveEndgameShuffleboard(endgame);
  private final DeployEndgame deployEndgame = new DeployEndgame(endgame);
  private final EndgameReadyUp endgameReadyUp = new EndgameReadyUp(endgame);
  private final EndgameToCenter endgameToCenter = new EndgameToCenter(endgame);

  // Scoring
  private final StopScoring scoringStop = new StopScoring(scoring);
  private final RotateWrist rotateWrist = new RotateWrist(scoring);
  private final FlipperOut flipperOut = new FlipperOut(scoring);
  private final RotateWristToReady rotateWristToReady = new RotateWristToReady(scoring);
  private final CloseClaw closeClaw = new CloseClaw(scoring);
  private final OpenClaw openClaw = new OpenClaw(scoring);

  // Limelight
  private final LimelightMoveToAprilTag goToTarget = new LimelightMoveToAprilTag(base, limelight);
  private final LimelightMoveToConeNode goToTargetTape = new LimelightMoveToConeNode(base, limelight);

  //Controller Ports (check in Driver Station, IDs may be different for each computer)
  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 1;  
  private static final int KStreamDeckPort = 2;
  private static final int KTestingStreamDeckPort = 3;

  //Deadzone
  private static final double KDeadZone = 0.05;
  
  //Joystick Axis IDs 
  private static final int KLeftYAxis = 1;
  private static final int KRightYAxis = 3;
  private static final int KLeftXAxis = 0;
  private static final int KRightXAxis = 2;

  //Logitech Button Constants
  public static final int KLogitechButtonX = 1;
  public static final int KLogitechButtonA = 2;
  public static final int KLogitechButtonB = 3;
  public static final int KLogitechButtonY = 4;
  public static final int KLogitechLeftBumper = 5;
  public static final int KLogitechRightBumper = 6;
  public static final int KLogitechLeftTrigger = 7;
  public static final int KLogitechRightTrigger = 8;

  //Xbox Button Constants
  public static final int KXboxButtonA = 1;
  public static final int KXboxButtonB = 2;
  public static final int KXboxButtonX = 3;
  public static final int KXboxButtonY = 4;
  public static final int KXboxLeftBumper = 5;
  public static final int KXboxRightBumper = 6;
  public static final int KXboxSelectButton = 7;
  public static final int KXboxStartButton = 8;
  public static final int KXboxLeftTrigger = 2;
  public static final int KXboxRightTrigger = 3;

  //Stream Deck Constants
  public static final int KConeModeButton = 1;
  public static final int KCubeModeButton = 2;
  public static final int KLiftLowSetpoint = 3;
  public static final int KLiftMidSetpoint = 4;
  public static final int KLiftHighSetpoint = 5;
  public static final int KCloseClawButton = 6;
  public static final int KOpenClawButton = 7;
  public static final int KMoveLiftUp = 8;
  public static final int KMoveLiftDown = 9;
  public static final int KLiftToWaitingPos = 10;

  public static final int KIntakeDown = 11;
  public static final int KIntakeUp = 12;
  public static final int KDefenseModeButton = 15;

  //Game Controllers
  public static Joystick logitech;
  public static Joystick streamDeck;
  public static Joystick testStreamDeck;
  public static XboxController xbox; 
  //Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button

  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

  public Trigger xboxBtnRT, xboxBtnLT;

  public JoystickButton coneModeButton, cubeModeButton, liftLowSetpointButton, liftMidSetpointButton, liftHighSetpointButton, closeClawButton, // Vjoy 1
    openClawButton, moveLiftUpButton, moveLiftDownButton, liftToWaitingPosButton, intakeUpButton, intakeDownButton, defenseModeButton;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton streamDeck1, streamDeck2, streamDeck3, streamDeck4, streamDeck5, streamDeck6, streamDeck7, streamDeck8, streamDeck9, // Vjoy 2
    streamDeck10, streamDeck11, streamDeck12, streamDeck13, streamDeck14, streamDeck15;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    base.setDefaultCommand(driveWithJoysticks);
    intake.setDefaultCommand(intakeStop);
    orientation.setDefaultCommand(OrientationMoveOut);
    scoring.setDefaultCommand(scoringStop);

    //Game controllers
    logitech = new Joystick(KLogitechPort); //Logitech Dual Action
    xbox = new XboxController(KXboxPort);   //Xbox 360 for Windows
    streamDeck = new Joystick(KStreamDeckPort);   //Stream Deck + vjoy
    testStreamDeck = new Joystick(KStreamDeckPort);   //Stream Deck + vjoy

    // Logitch Buttons 
    logitechBtnX = new JoystickButton(logitech, KLogitechButtonX);
    logitechBtnA = new JoystickButton(logitech, KLogitechButtonA);
    logitechBtnB = new JoystickButton(logitech, KLogitechButtonB);
    logitechBtnY = new JoystickButton(logitech, KLogitechButtonY);
    logitechBtnLB = new JoystickButton(logitech, KLogitechLeftBumper);
    logitechBtnRB = new JoystickButton(logitech, KLogitechRightBumper);
    logitechBtnLT = new JoystickButton(logitech, KLogitechLeftTrigger);
    logitechBtnRT = new JoystickButton(logitech, KLogitechRightTrigger);

    // XBox Buttons
    xboxBtnA = new JoystickButton(xbox, KXboxButtonA);
  	xboxBtnB = new JoystickButton(xbox, KXboxButtonB);
		xboxBtnX = new JoystickButton(xbox, KXboxButtonX);
		xboxBtnY = new JoystickButton(xbox, KXboxButtonY);
		xboxBtnLB = new JoystickButton(xbox, KXboxLeftBumper);
    xboxBtnRB = new JoystickButton(xbox, KXboxRightBumper);
    xboxBtnSelect = new JoystickButton(xbox, KXboxSelectButton);
		xboxBtnStrt = new JoystickButton(xbox, KXboxStartButton);
    xboxBtnLT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxLeftTrigger))));
    xboxBtnRT = new Trigger(() -> (joystickThreshold(xbox.getRawAxis(KXboxRightTrigger))));

    coneModeButton = new JoystickButton(streamDeck, KConeModeButton);
    cubeModeButton = new JoystickButton(streamDeck, KCubeModeButton);
    liftLowSetpointButton = new JoystickButton(streamDeck, KLiftLowSetpoint);
    liftMidSetpointButton = new JoystickButton(streamDeck, KLiftMidSetpoint);
    liftHighSetpointButton = new JoystickButton(streamDeck, KLiftHighSetpoint);
    closeClawButton = new JoystickButton(streamDeck, KCloseClawButton);
    openClawButton = new JoystickButton(streamDeck, KOpenClawButton);
    moveLiftUpButton = new JoystickButton(streamDeck, KMoveLiftUp);
    moveLiftDownButton = new JoystickButton(streamDeck, KMoveLiftDown);
    liftToWaitingPosButton = new JoystickButton(streamDeck, KLiftToWaitingPos);
    intakeDownButton = new JoystickButton(streamDeck, KIntakeDown);
    intakeUpButton = new JoystickButton(streamDeck, KIntakeUp);
    defenseModeButton = new JoystickButton(streamDeck, KDefenseModeButton);

    streamDeck1 = new JoystickButton(testStreamDeck, 1);
    streamDeck2 = new JoystickButton(testStreamDeck, 2);
    streamDeck3 = new JoystickButton(testStreamDeck, 3);
    streamDeck4 = new JoystickButton(testStreamDeck, 4);
    streamDeck5 = new JoystickButton(testStreamDeck, 5);
    streamDeck6 = new JoystickButton(testStreamDeck, 6);
    streamDeck7 = new JoystickButton(testStreamDeck, 7);
    streamDeck8 = new JoystickButton(testStreamDeck, 8);
    streamDeck9 = new JoystickButton(testStreamDeck, 9);
    streamDeck10 = new JoystickButton(testStreamDeck, 10);
    streamDeck11 = new JoystickButton(testStreamDeck, 11);
    streamDeck12 = new JoystickButton(testStreamDeck, 12);
    streamDeck13 = new JoystickButton(testStreamDeck, 13);
    streamDeck14 = new JoystickButton(testStreamDeck, 14);
    streamDeck15 = new JoystickButton(testStreamDeck, 15);
  	
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // logitechBtnX.whileTrue(goToTarget);
    // logitechBtnB.whileTrue(goToTargetTape);
    logitechBtnLB.onTrue(toggleFastSpeed);
    logitechBtnLB.onFalse(toggleSlowSpeed);

    logitechBtnY.onTrue(resetGyro);
    
    coneModeButton.onTrue(setConeMode);
    cubeModeButton.onTrue(setCubeMode);
    defenseModeButton.onTrue(new ToggleDefenseMode(intake));

    streamDeck1.whileTrue(intakeSpinForward);
    streamDeck2.whileTrue(intakeSpinReverse);
    streamDeck3.whileTrue(moveSwivelUp);
    streamDeck4.whileTrue(moveSwivelDown);
    streamDeck5.whileTrue(OrientationMoveOut);
    streamDeck6.whileTrue(OrientationMoveIn);
    streamDeck7.whileTrue(orientationSpinOutwards);
    streamDeck8.whileTrue(orientationSpinInwards);
    streamDeck9.onTrue(rotateWrist);
    streamDeck10.whileTrue(flipperOut);
    streamDeck11.onTrue(openClaw);
    streamDeck12.onTrue(closeClaw);
    streamDeck13.whileTrue(setCubeMode);
    streamDeck14.whileTrue(setConeMode);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public static double scaleBetween(double unscaledNum, double minAllowed, double maxAllowed, double min, double max) {
    return (maxAllowed - minAllowed) * (unscaledNum - min) / (max - min) + minAllowed;
  }
       
  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("getLogiRightYAxis", -Y);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    SmartDashboard.putNumber("getLogiLeftYAxis", -Y);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0; 
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    SmartDashboard.putNumber("getLogiRightXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0; 
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    SmartDashboard.putNumber("getLogiLeftXAxis", -X);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if(X > KDeadZone || X < -KDeadZone)
      return X;
    else 
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }
public boolean joystickThreshold(double triggerValue) {
    if (Math.abs(triggerValue) < .09) 
      return false;
    else 
      return true;
  }
}
