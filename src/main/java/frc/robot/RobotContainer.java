// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Base.DriveWithJoysticks;
import frc.robot.commands.Base.ResetEncoders;
import frc.robot.commands.Endgame.*;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.Orientation;
import frc.robot.commands.Orientation.OrientationMoveAllForward;
import frc.robot.commands.Orientation.OrientationMoveAllReverse;
import frc.robot.commands.Orientation.OrientationSpinOnlyLeftandRightForward;
import frc.robot.commands.Orientation.OrientationSpinOnlyLeftandRightReverse;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.Limelight.LimelightMoveToAprilTag;
import frc.robot.commands.Limelight.LimelightMoveToConeNode;
import frc.robot.commands.Limelight.ToggleLimelightPipeline;
import frc.robot.commands.Orientation.OrientationStopOnlyExtension;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionForward;
import frc.robot.commands.Orientation.OrientationMoveOnlyExtensionReverse;
import frc.robot.commands.Base.ToggleSpeed;
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
  // private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);
  // private final ToggleSpeed toggleFastSpeed = new ToggleSpeed(base, KBaseDriveMaxPercent);
  // private final ToggleSpeed toggleSlowSpeed = new ToggleSpeed(base, KBaseDriveLowPercent);

  // Intake
  private final IntakeSpin intakeForward = new IntakeSpin(intake);
  private final IntakeStop intakeStop = new IntakeStop(intake);

  //Orientation
  private final OrientationMoveOnlyExtensionForward OrientationFoward1 = new OrientationMoveOnlyExtensionForward(orientation);
  private final OrientationMoveOnlyExtensionReverse OrientationBackward1 = new OrientationMoveOnlyExtensionReverse(orientation);
  private final OrientationStopOnlyExtension OrientationStop = new OrientationStopOnlyExtension(orientation);

  // Endgame
  private final MoveEndgameShuffleboard moveEndgameShuffleboard = new MoveEndgameShuffleboard(endgame);
  private final DeployEndgame deployEndgame = new DeployEndgame(endgame);
  private final EndgameReadyUp endgameReadyUp = new EndgameReadyUp(endgame);
  private final EndgameToCenter endgameToCenter = new EndgameToCenter(endgame);

  // Limelight
  private final LimelightMoveToAprilTag goToTarget = new LimelightMoveToAprilTag(base, limelight);
  private final LimelightMoveToConeNode goToTargetTape = new LimelightMoveToConeNode(base, limelight);

  //Controller Ports (check in Driver Station, IDs may be different for each computer)
  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 1;  

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

  //Game Controllers
  public static Joystick logitech;
  public static XboxController xbox; 
  //Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button
  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;
  public Trigger xboxBtnRT, xboxBtnLT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // base.setDefaultCommand(driveWithJoysticks);
    intake.setDefaultCommand(intakeStop);

    //Game controllers
    logitech = new Joystick(KLogitechPort); //Logitech Dual Action
    xbox = new XboxController(KXboxPort);   //Xbox 360 for Windows

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
    // logitechBtnY.onTrue(new ResetEncoders(base));
    // logitechBtnLB.onTrue(toggleFastSpeed);
    // logitechBtnLB.onFalse(toggleSlowSpeed);
    // logitechBtnA.onTrue(new ToggleLimelightPipeline(limelight));

    logitechBtnX.whileTrue(goToTarget);
    logitechBtnB.whileTrue(goToTargetTape);
    
    xboxBtnA.onTrue(intakeForward);
  ;
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
