package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput; // Sensors

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;

/* 
 * 3 motors - 2 Neos or 775s & 1 Snow Blower - most likely TalonSRX, but can be VictorSPX or Jaguar (hopefully not that)
 * 1 encoder for extension
 * 3 sensors most likely
 */
public class Orientation extends SubsystemBase {
    private CANSparkMax orientationLeftMotor;
    private CANSparkMax orientationRightMotor;
    private TalonSRX orientationMotorExtension;

    private PIDController extensionController;
    private double extensionControllerkP = 0.00001;
    private double extensionControllerkI = 0;
    private double extensionControllerkD = 0;
    
    private DigitalInput DoorControl;
    private DigitalInput BaseChecker;
    private DigitalInput TipChecker;
    private DigitalInput HallEffectSensor1;
    private DigitalInput HallEffectSensor2;

    private boolean orientationMode;
    
    public ORIENTATIONSTATE OrientationState = ORIENTATIONSTATE.CUBE;

    public Orientation() {

        orientationLeftMotor = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        orientationRightMotor = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        orientationMotorExtension = new TalonSRX(KOrientationMotorExtensionID);
        
        DoorControl = new DigitalInput (KOrientationkDoorControlID);
        BaseChecker = new DigitalInput (KOrientationkBaseCheckerID);
        TipChecker = new DigitalInput (KOrientationkTipCheckerID);
        HallEffectSensor1 = new DigitalInput(KOrientationHallEffectSensor1ID);
        HallEffectSensor2 = new DigitalInput(KOrientationHallEffectSensor2ID);

        extensionController = new PIDController(extensionControllerkP, extensionControllerkI, extensionControllerkD);
        //orientationMotorExtension.getSelectedSensorPosition();

        orientationLeftMotor.setIdleMode(IdleMode.kBrake);
        // orientationMotorExtension.setIdleMode(IdleMode.kBrake);
        
        orientationRightMotor.follow(orientationLeftMotor);
    }

    public void moveOrientationLeftandRightMotors() {
        if (orientationMode) {
            orientationLeftMotor.set(KCubeLeftandRightMotorSpeeds);
        }
        else if (!orientationMode) {
            orientationLeftMotor.set(KConeLeftandRightMotorSpeeds);
        }
    }

    public void moveOrientationLeftandRightMotors(double speed) {
        orientationLeftMotor.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        orientationMotorExtension.set(ControlMode.PercentOutput, speed);
    }

    public void setCubeMode() {
        orientationMode = false;
    }

    public void setConeMode() {
        orientationMode = true; 
    }

    public boolean isConeMode() {
        return orientationMode;
    }

    public void stopOrientationLeftandRightMotors() {
        orientationLeftMotor.set(0);
    }

    public void stopOrientationMotorExtension() {
        orientationMotorExtension.set(ControlMode.PercentOutput, 0);
    }

    public Boolean getDoorSensor() {
        return DoorControl.get();
    }

    public Boolean getBaseSensor() {
        return BaseChecker.get();
    }

    public Boolean getTipSensor() {
        return TipChecker.get();
    }

    public Boolean getHallEffectSensor1() {
        return HallEffectSensor1.get();
    }

    public Boolean getHallEffectSensor2(){
        return HallEffectSensor2.get();
    }

}
