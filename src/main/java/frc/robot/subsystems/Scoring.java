package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid; // Pnuematics
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Pnuematics
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics


public class Scoring extends SubsystemBase{
    private CANSparkMax claw;
    private CANSparkMax wrist;
    private CANSparkMax flipper;
    private CANSparkMax lift;

    public Scoring() {
        claw = new CANSparkMax(KClawMotor, MotorType.kBrushless);
        wrist = new CANSparkMax(KWristMotor, MotorType.kBrushless);
        flipper = new CANSparkMax(KFlipperMotor, MotorType.kBrushless);
        lift = new CANSparkMax(KLiftMotor, MotorType.kBrushless);
    }

    public void moveClaw(double speed) {
        claw.set(speed);
    }
    public void moveWrist(double speed) {
        wrist.set(speed);
    }
    public void moveFlipper(double speed) {
        flipper.set(speed);
    }
    public void moveLift(double speed) {
        lift.set(speed);
    }

    public void stop() {
        claw.set(0);
        wrist.set(0);
        flipper.set(0);
        lift.set(0);
    }       
}
