package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.revrobotics.CANSparkMax; // Covers Neo's
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Cover's Neo's
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput; // Sensors

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;

/* 
 * 3 motors - 2 Neos or 775s & 1 Snow Blower
 * 1 encoder for extension
 * 3 sensors most likely
 */
public class Orientation extends SubsystemBase {
    private CANSparkMax orientationLeftMotor;
    private CANSparkMax orientationRightMotor;
    private VictorSPX orientationMotorExtension;

    private PIDController extensionController;
    private double extensionControllerkP = 0.00001;
    private double extensionControllerkI = 0;
    private double extensionControllerkD = 0;
    
    private DigitalInput IRSensor1;
    private DigitalInput IRSensor2;
    private DigitalInput IRSensor3;
    
    
    public Orientation() {
        orientationLeftMotor = new CANSparkMax(KOrientationLeftMotorID, MotorType.kBrushless);
        orientationRightMotor = new CANSparkMax(KOrientationRightMotorID, MotorType.kBrushless);

        orientationMotorExtension = new VictorSPX(KOrientationMotorExtensionID);
        
        IRSensor1 = new DigitalInput (KOrientationSensor1ID);
        IRSensor2 = new DigitalInput (KOrientationSensor2ID);
        IRSensor3 = new DigitalInput (KOrientationSensor3ID);

        extensionController = new PIDController(extensionControllerkP, extensionControllerkI, extensionControllerkD);
        //orientationMotorExtension.getSelectedSensorPosition();

        orientationRightMotor.follow(orientationLeftMotor);
    }

    public void moveOrientationLeftandRightMotors(double speed) {
        orientationLeftMotor.set(speed);
    }

    public void moveOrientationMotorExtension(double speed) {
        orientationMotorExtension.set(ControlMode.PercentOutput, speed);
    }

    public void stopOrientationLeftandRightMotors() {
        orientationLeftMotor.set(0);
    }

    public void stopOrientationMotorExtension() {
        orientationMotorExtension.set(ControlMode.PercentOutput, 0);
    }

    public Boolean getSensor1() {
        return IRSensor1.get();
    }

    public Boolean getSensor2() {
        return IRSensor2.get();
    }

    public Boolean getSensor3() {
        return IRSensor3.get();
    }

    public double getEncoder() {
        return orientationMotorExtension.getSelectedSensorPosition();
      }

    public void extensionPosition(double setPoint) {
        moveOrientationMotorExtension(extensionController.calculate(getEncoder(),setPoint));
    }

}
