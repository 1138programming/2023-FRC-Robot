package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Neos and 775
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Covers Neos and 775 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid; // Pnuematics
import edu.wpi.first.wpilibj.PneumaticsModuleType; // Pnuematics
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Pnuematics


public class Scoring extends SubsystemBase{
    private CANSparkMax clawMotor1;
    private CANSparkMax clawMotor2;
    private CANSparkMax angleArmMotor;
    private CANSparkMax extensionMotor1;
    private CANSparkMax extensionMotor2;
    private DoubleSolenoid clawSolenoid;

    public Scoring() {
        clawMotor1 = new CANSparkMax(KClawMotor1, MotorType.kBrushless);
        clawMotor2 = new CANSparkMax(KClawMotor2, MotorType.kBrushless);
        angleArmMotor = new CANSparkMax(KAngleArmMotor, MotorType.kBrushless);
        extensionMotor1 = new CANSparkMax(KExtensionMotor1, MotorType.kBrushless);
        extensionMotor2 = new CANSparkMax(KExtensionMotor2, MotorType.kBrushless);

        extensionMotor1.setInverted(KExtensionMotor1Reversed);

        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, KCLawSolenoidForwardChannel, KCLawSolenoidReverseChannel);
    }

    public void moveClawMotors(double speed) {
        clawMotor1.set(speed);
        clawMotor2.set(speed);
    }

    public void moveAngleArmMotor(double speed) {
        angleArmMotor.set(speed);
    }

    public void moveExtensionMotors(double speed) {
        extensionMotor1.set(speed);
        extensionMotor2.set(speed);
    }

    public void stopClawMotors() {
        clawMotor1.set(0);
        clawMotor2.set(0);
    }

    public void stopAngleArmMotor() {
        angleArmMotor.set(0);
    }

    public void stopExtensionMotors() {
        extensionMotor1.set(0);
        extensionMotor2.set(0);
    }
    
    public void cLawSolenoidForward() {
        clawSolenoid.set(kForward);
    }
    
    public void cLawSolenoidReverse() {
        clawSolenoid.set(kReverse);
    }
    
}
