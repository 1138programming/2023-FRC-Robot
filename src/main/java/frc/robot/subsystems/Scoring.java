package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid; // Pnuematics
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Pnuematics
import edu.wpi.first.wpilibj.Servo;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics
import edu.wpi.first.math.controller.PIDController;

public class Scoring extends SubsystemBase{
    private CANSparkMax flipper;
    private CANSparkMax lift;
    private Servo claw;
    private Servo wrist;

    private DigitalInput BaseChecker;
    private DigitalInput TipChecker;

    private RelativeEncoder flipperEncoder;
    private PIDController flipperController;

    private PIDController liftControl;
    private RelativeEncoder liftEncoder;

    private boolean scoringMode;

    public Scoring() {
        flipper = new CANSparkMax(KFlipperMotor, MotorType.kBrushless);
        lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
        claw = new Servo(KClawServo);
        wrist = new Servo (KWristServo);

        flipperEncoder = flipper.getEncoder();

        flipperController = new PIDController(KFlipperP, KFlipperI, KFlipperD);
        liftControl = new PIDController(KLiftP, KLiftI, KLiftD);
        liftEncoder = lift.getEncoder();

        BaseChecker = new DigitalInput (KOrientationkBaseCheckerID);
        TipChecker = new DigitalInput (KOrientationkTipCheckerID);
    }

    public void moveFlipper(double speed) {
        flipper.set(speed);
    }
    public void moveLift(double setPoint) {
        lift.set(liftControl.calculate(liftEncoder.getPosition(),setPoint));
    }
    public void moveClaw(double setpoint) {
        setpoint *= 1/135;
        claw.set(setpoint);
    }
    public void moveWrist(double setpoint) {
        setpoint *= 1/300;
        wrist.set(setpoint);
    }
    
    public void closeClaw() {
        if (scoringMode) {
            claw.set(KCloseClawCone);
        }
        else {
            claw.set(KCloseClawCube);
        }
    }

    public double getFlipperPos(){
        return flipperEncoder.getPosition();
    }

    public double getLiftPos() {
        return liftEncoder.getPosition();
    }

    public void flipToPos(double setPoint) {
        moveFlipper(flipperController.calculate(getFlipperPos(), setPoint));
    }
    
    public boolean getBaseSensor() {
        return BaseChecker.get();
    }

    public boolean getTipSensor() {
        return TipChecker.get();
    }

    public void setCubeMode() {
        scoringMode = false;
    }

    public void setConeMode() {
        scoringMode = true; 
    }

    public boolean isConeMode() {
        return scoringMode;
    }

    public void stop() {
        claw.set(0);
        wrist.set(0);
        flipper.set(0);
        lift.set(0);
    }       
}
