// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandGroups.Auton.ScoreHighAndLeave;
import frc.robot.CommandGroups.Auton.ScoreHighDontMove;
import frc.robot.CommandGroups.IntakeThenLift.LiftHighSetpoint;
import frc.robot.CommandGroups.IntakeThenLift.LiftLowSetpoint;
import frc.robot.CommandGroups.IntakeThenLift.LiftMidSetpoint;
import frc.robot.CommandGroups.BackThenForward;
import frc.robot.CommandGroups.ScoreLowDontMove;
import frc.robot.commands.Base.AutoBalance;
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Intake.IntakeBottomNoCollect;
import frc.robot.commands.Intake.IntakeMoveSwivelDown;
import frc.robot.commands.Intake.IntakeMoveSwivelUp;
import frc.robot.commands.Intake.IntakeShootOut;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeSpinAndSwivel;
import frc.robot.commands.Intake.IntakeSpinReverse;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.SetConeMode;
import frc.robot.commands.Intake.SetCubeMode;

import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Orientation;
import frc.robot.subsystems.Limelight;
//Commands for the Base
import frc.robot.commands.Base.ToggleSpeed;
import frc.robot.commands.Base.ResetEncoders;
import frc.robot.commands.Base.ResetGyro;
import frc.robot.commands.Base.ToggleDefenseMode;
import frc.robot.commands.Base.SetDefenseModeFalse;
import frc.robot.commands.Base.SetDefenseModeTrue;
import frc.robot.commands.Base.DriveWithJoysticks;
//Commands for the Intake
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSwivelBottom;
import frc.robot.commands.Intake.IntakeSwivelShoot;
import frc.robot.commands.Intake.IntakeSwivelTop;
import frc.robot.commands.Intake.OuttakeAndSwivel;
//Commands for the Orientation
import frc.robot.commands.Orientation.OrientationSpinOut;
import frc.robot.commands.Orientation.OrientationSpinIn;
import frc.robot.commands.Orientation.OrientationStop;
import frc.robot.commands.Orientation.CheckDoorAndCollectObject;
import frc.robot.commands.Orientation.ExtendAndIntake;
import frc.robot.commands.Orientation.ExtendAndOuttake;
import frc.robot.commands.Scoring.Claw.CloseClaw;
import frc.robot.commands.Scoring.Claw.OpenClaw;
import frc.robot.commands.Scoring.Claw.RotateWrist;
import frc.robot.commands.Scoring.Claw.RotateWristCube;
import frc.robot.commands.Scoring.Claw.RotateWristOut;
import frc.robot.commands.Scoring.Claw.RotateWristToReady;
import frc.robot.commands.Scoring.Lift.FlipperIn;
import frc.robot.commands.Scoring.Lift.FlipperOut;
import frc.robot.commands.Scoring.Lift.InnerLiftIn;
import frc.robot.commands.Scoring.Lift.InnerLiftOut;
import frc.robot.commands.Scoring.Lift.LiftStop;
import frc.robot.commands.Scoring.Lift.MoveFlipper;
import frc.robot.commands.Scoring.Lift.MoveInnerLift;
import frc.robot.commands.Scoring.Lift.MoveLift;
import frc.robot.commands.Scoring.Lift.MoveLiftToHighPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToLowPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToMidPos;
import frc.robot.commands.Scoring.Lift.MoveLiftToReadyPos;
import frc.robot.commands.Orientation.ExtensionNudge;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionForward;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionReverse;

import frc.robot.commands.Limelight.LimelightMoveToAprilTag;
import frc.robot.commands.Limelight.LimelightMoveToConeNode;
import frc.robot.commands.Limelight.ToggleLimelightPipeline;

import frc.robot.CommandGroups.BackThenForward;

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
  private final Lift lift = new Lift();
  private final Claw claw = new Claw();
  private final Endgame endgame = new Endgame();
  private final Intake intake = new Intake();
  private final Orientation orientation = new Orientation();
  private final Limelight limelight = new Limelight();

  // Base
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  private final ToggleSpeed toggleMaxSpeed = new ToggleSpeed(base, KBaseDriveMaxPercent);
  private final ToggleSpeed toggleMidSpeed = new ToggleSpeed(base, KBaseDriveMidPercent);
  private final ToggleSpeed toggleLowSpeed = new ToggleSpeed(base, KBaseDriveLowPercent);
  private final ResetGyro resetGyro = new ResetGyro(base);
  private final ResetEncoders resetEncoders = new ResetEncoders(base);
  private final ToggleDefenseMode toggleDefenseMode = new ToggleDefenseMode(base);
  private final SetDefenseModeTrue setDefenseModeTrue = new SetDefenseModeTrue(base);
  private final SetDefenseModeFalse setDefenseModeFalse = new SetDefenseModeFalse(base);

  // Intake  
  private final IntakeSpin intakeSpinForward = new IntakeSpin(intake);
  private final IntakeSpinReverse intakeSpinReverse = new IntakeSpinReverse(intake);
  private final IntakeShootOut intakeShootOut = new IntakeShootOut(intake);

  private final IntakeMoveSwivelDown moveSwivelDown = new IntakeMoveSwivelDown(intake);
  private final IntakeMoveSwivelUp moveSwivelUp = new IntakeMoveSwivelUp(intake);

  private final IntakeSwivelTop intakeSwivelTop = new IntakeSwivelTop(intake);
  private final IntakeSwivelBottom intakeSwivelBottom = new IntakeSwivelBottom(intake);
  private final IntakeSwivelShoot intakeSwivelShoot = new IntakeSwivelShoot(intake);
  private final IntakeBottomNoCollect intakeBottomNoCollect = new IntakeBottomNoCollect(intake);

  private final IntakeSpinAndSwivel intakeSpinAndSwivel = new IntakeSpinAndSwivel(intake);
  private final OuttakeAndSwivel outtakeAndSwivel = new OuttakeAndSwivel(intake);

  private final IntakeStop intakeStop = new IntakeStop(intake);
  private final SetConeMode setConeMode = new SetConeMode(orientation, intake, claw, limelight);
  private final SetCubeMode setCubeMode = new SetCubeMode(orientation, intake, claw, limelight);
  

  //Orientation
  private final ExtendAndOuttake extendOut = new ExtendAndOuttake(orientation);
  private final ExtendAndIntake storeObject = new ExtendAndIntake(orientation);
  private final ExtensionNudge nudge = new ExtensionNudge(orientation);
  private final OrientationSpinIn orientationSpinIn = new OrientationSpinIn(orientation);
  private final OrientationSpinOut orientationSpinOut = new OrientationSpinOut(orientation);
  private final CheckDoorAndCollectObject checkDoorAndCollectObject = new CheckDoorAndCollectObject(orientation);
  private final OrientationStop orientationStop = new OrientationStop(orientation);

  // Endgame
  private final MoveEndgameShuffleboard moveEndgameShuffleboard = new MoveEndgameShuffleboard(endgame);
  private final DeployEndgame deployEndgame = new DeployEndgame(endgame);
  private final EndgameReadyUp endgameReadyUp = new EndgameReadyUp(endgame);
  private final EndgameToCenter endgameToCenter = new EndgameToCenter(endgame);

  // Scoring
  private final LiftStop liftstop = new LiftStop(lift);
  private final RotateWrist rotateWrist = new RotateWrist(claw);
  private final RotateWristCube rotateWristCube = new RotateWristCube(claw);
  private final FlipperOut flipperOut = new FlipperOut(lift);
  private final FlipperIn flipperIn = new FlipperIn(lift);
  private final MoveFlipper moveFlipperForward = new MoveFlipper(lift, 0.1);
  private final MoveFlipper moveFlipperReverse = new MoveFlipper(lift, -0.1);
  private final RotateWristToReady rotateWristToReady = new RotateWristToReady(claw);
  private final CloseClaw closeClaw = new CloseClaw(claw);
  private final OpenClaw openClaw = new OpenClaw(claw);
  private final MoveLift moveLiftUp = new MoveLift(lift, 0.1);
  private final MoveLift moveLiftDown = new MoveLift(lift, -0.1);
  // private final MoveLift moveLiftUp = new MoveLift(lift, 0.4);
  // private final MoveLift moveLiftDown = new MoveLift(lift, -0.4);
  private final MoveInnerLift moveInnerLiftUp = new MoveInnerLift(lift, 0.2);
  private final MoveInnerLift moveInnerLiftDown = new MoveInnerLift(lift, -0.2);
  private final MoveLiftToHighPos moveLiftToHighPos = new MoveLiftToHighPos(lift);
  private final MoveLiftToMidPos moveLiftToMidPos = new MoveLiftToMidPos(lift);
  private final MoveLiftToLowPos moveLiftToLowPos = new MoveLiftToLowPos(lift);
  private final MoveLiftToReadyPos moveLiftToReadyPos = new MoveLiftToReadyPos(lift);
  // Limelight
  private final LimelightMoveToAprilTag goToTarget = new LimelightMoveToAprilTag(base, limelight);
  private final LimelightMoveToConeNode goToTargetTape = new LimelightMoveToConeNode(base, limelight);

  private final LiftHighSetpoint liftHighSetpoint = new LiftHighSetpoint(lift, claw);
  private final LiftMidSetpoint liftMidSetpoint = new LiftMidSetpoint(lift, claw);
  private final LiftLowSetpoint liftLowSetpoint = new LiftLowSetpoint(lift, claw);





  //Controller Ports (check in Driver Station, IDs may be different for each computer)
  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 1;  
  private static final int KStreamDeckPort = 2;
  private static final int KTestingStreamDeckPort = 3;
  private static final int KTuningStreamDeckPort = 4;


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
  public static Joystick compStreamDeck;
  public static Joystick testStreamDeck;
  public static Joystick tuningStreamDeck;
  public static XboxController xbox; 
  //Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button

  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;

  public Trigger xboxBtnRT, xboxBtnLT;

  public JoystickButton coneModeButton, cubeModeButton, liftLowSetpointButton, liftMidSetpointButton, liftHighSetpointButton, closeClawButton, // Vjoy 1
    openClawButton, moveLiftUpButton, moveLiftDownButton, liftToWaitingPosButton, intakeUpButton, intakeDownButton, liftResetButton, defenseModeButton;
  
  public JoystickButton comp1, comp2, comp3, comp4, comp5, comp6, comp7, comp8, comp9, comp10, comp11, comp12, comp13, comp14;

  // Top Left SD = 1, numbered from left to right
  public JoystickButton streamDeck1, streamDeck2, streamDeck3, streamDeck4, streamDeck5, streamDeck6, streamDeck7, streamDeck8, streamDeck9, // Vjoy 2
    streamDeck10, streamDeck11, 
    streamDeck12, streamDeck13, streamDeck14, streamDeck15;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    base.setDefaultCommand(driveWithJoysticks);
    // base.setDefaultCommand(new BaseStop(base));
    intake.setDefaultCommand(intakeStop);
    orientation.setDefaultCommand(orientationStop);
    lift.setDefaultCommand(liftstop);

    //Game controllers
    logitech = new Joystick(KLogitechPort); //Logitech Dual Action
    xbox = new XboxController(KXboxPort);   //Xbox 360 for Windows
    compStreamDeck = new Joystick(KStreamDeckPort);   //Stream Deck + vjoy
    testStreamDeck = new Joystick(KTestingStreamDeckPort);   //Stream Deck + vjoy
    tuningStreamDeck = new Joystick(KTuningStreamDeckPort);   //Stream Deck + vjoy

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

    coneModeButton = new JoystickButton(compStreamDeck, KConeModeButton);
    cubeModeButton = new JoystickButton(compStreamDeck, KCubeModeButton);
    liftLowSetpointButton = new JoystickButton(compStreamDeck, KLiftLowSetpoint);
    liftMidSetpointButton = new JoystickButton(compStreamDeck, KLiftMidSetpoint);
    liftHighSetpointButton = new JoystickButton(compStreamDeck, KLiftHighSetpoint);
    closeClawButton = new JoystickButton(compStreamDeck, KCloseClawButton);
    openClawButton = new JoystickButton(compStreamDeck, KOpenClawButton);
    moveLiftUpButton = new JoystickButton(compStreamDeck, KMoveLiftUp);
    moveLiftDownButton = new JoystickButton(compStreamDeck, KMoveLiftDown);
    liftToWaitingPosButton = new JoystickButton(compStreamDeck, KLiftToWaitingPos);
    intakeDownButton = new JoystickButton(compStreamDeck, KIntakeDown);
    intakeUpButton = new JoystickButton(compStreamDeck, KIntakeUp);
    
    liftResetButton = new JoystickButton(compStreamDeck, 14);
    
    defenseModeButton = new JoystickButton(compStreamDeck, KDefenseModeButton);

    comp1 = new JoystickButton(compStreamDeck, 1);
    comp2 = new JoystickButton(compStreamDeck, 2);
    comp3 = new JoystickButton(compStreamDeck, 3);
    comp4 = new JoystickButton(compStreamDeck, 4);
    comp5 = new JoystickButton(compStreamDeck, 5);
    comp6 = new JoystickButton(compStreamDeck, 6);
    comp7 = new JoystickButton(compStreamDeck, 7);
    comp8 = new JoystickButton(compStreamDeck, 8);
    comp9 = new JoystickButton(compStreamDeck, 9);
    comp10 = new JoystickButton(compStreamDeck, 10);
    comp11 = new JoystickButton(compStreamDeck, 11);
    comp12 = new JoystickButton(compStreamDeck, 12);
    comp13 = new JoystickButton(compStreamDeck, 13);
    comp14 = new JoystickButton(compStreamDeck, 14);

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
    logitechBtnLB.onTrue(toggleMaxSpeed);
    logitechBtnLB.onFalse(toggleMidSpeed);

    // logitechBtnRB.onTrue(setDefenseModeTrue);
    // logitechBtnRB.onFalse(setDefenseModeFalse);
    // logitechBtnRB.onTrue(toggleLowSpeed);
    // logitechBtnLB.or(logitechBtnRB).onFalse(toggleMidSpeed);

    logitechBtnY.onTrue(resetEncoders);

    // liftHighSetpointButton.whileTrue(new MoveLiftToHighPos(lift));
    // liftMidSetpointButton.whileTrue(new MoveLiftToMidPos(lift));
    // liftLowSetpointButton.whileTrue(new MoveLiftToLowPos(lift));
    // liftResetButton.whileTrue(new MoveLiftToReadyPos(lift));

    // moveLiftUpButton.whileTrue(new MoveLift(lift, 0.3));
    // moveLiftDownButton.whileTrue(new MoveLift(lift, -0.3));

    comp1.onTrue(intakeSwivelShoot);
    comp2.whileTrue(intakeShootOut);

    // comp3.onTrue(liftLowSetpoint);
    // comp4.onTrue(liftMidSetpoint);
    // comp5.onTrue(liftHighSetpoint);
    comp6.onTrue(closeClaw);
    comp7.onTrue(openClaw);

    comp8.whileTrue(moveLiftUp);
    comp9.whileTrue(moveLiftDown);
    comp10.onTrue(intakeBottomNoCollect);
    comp11.whileTrue(intakeSwivelBottom);
    comp11.onFalse(intakeSwivelTop);

    comp12.whileTrue(moveSwivelUp);

    comp13.onTrue(moveLiftToReadyPos);

    // comp14.onTrue(toggleDefenseMode);

    streamDeck1.whileTrue(moveSwivelUp);
    streamDeck2.whileTrue(moveSwivelDown);
    streamDeck3.whileTrue(intakeSpinForward);
    streamDeck4.whileTrue(moveLiftUp);
    streamDeck5.whileTrue(moveLiftDown);
    streamDeck6.whileTrue(moveInnerLiftUp);
    streamDeck7.whileTrue(intakeShootOut);
    streamDeck8.whileTrue(moveFlipperForward);
    streamDeck9.whileTrue(moveFlipperReverse);
    streamDeck10.whileTrue(rotateWrist);
    streamDeck11.whileTrue(rotateWristCube);
    streamDeck12.onTrue(extendOut);
    streamDeck13.onTrue(storeObject);
    streamDeck14.whileTrue(intakeSpinReverse);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */ 
  public Command getAutonomousCommand()
   {
    // return null;
    return new BackThenForward(base);
    // return new ScoreLowDontMove(base);
    // return new ScoreHighDontMove(lift, claw, base, intake);
    // return new ScoreHighAndLeave(lift, claw, base, intake);
    // return new AutoBalance(base);
  }

  public static double scaleBetween(double unscaledNum, double minAllowed, double maxAllowed, double min, double max) {
    return (maxAllowed - +minAllowed) * (unscaledNum - min) / (max - min) + minAllowed;
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
