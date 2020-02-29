
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

//import frc.robot.sensors.Limelight;
import frc.robot.sensors.Pipeline;



public class ColorWheel {
    //--------------------------------\\
    // Variables:
    double spinnerClockwise = 0.0; // Value to be set after testing.
    double spinnerCounterclockwise = 0.0; // Value to be set after testing.
    double handRaise = 0.0; // Value to be set after testing.
    double handLower = 0.0; // Value to be set after testing.
    //--------------------------------\\
    //--------------------------------\\
    // Controller:
    public XboxController controller1 = new XboxController(1); //Really Controller #2 But Base Zero so it is called "1".
    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;
    //--------------------------------\\
    //Color Wheel Section:
    public WPI_TalonSRX Hand;        // Will Likely Undergo Change.
    public WPI_TalonSRX Spinner;     // Will Likely Undergo Change.
    //--------------------------------\\

    //------------------------------------------------\\
    public void clockwise() {

        if (controller1.getTriggerAxis(kLeft) > 0.5) {
            if (controller1.getAButtonPressed()) {
                Spinner.set(spinnerClockwise);
            }
        // Failsafe:
        else { Spinner.set(0.0); }
        }
    }
    //------------------------------------------------\\
    public void counterclockwise() {
        if (controller1.getTriggerAxis(kLeft) > 0.5) {
            if (controller1.getBButtonPressed()) {
                Spinner.set(spinnerCounterclockwise);
            }
        // Failsafe:
        else { Spinner.set(0.0); }
        }
    }
    //------------------------------------------------\\
    public void raise() {
        if (controller1.getTriggerAxis(kLeft) > 0.5) {
            if (controller1.getXButtonPressed()) {
                Hand.set(handRaise);
            }
        // Failsafe:
        else { Hand.set(0.0); }
        }
    }
    //------------------------------------------------\\
    public void unraise() {
        if (controller1.getTriggerAxis(kLeft) > 0.5) {
            if (controller1.getYButtonPressed()) {
                Hand.set(handLower);
            }
        // Failsafe:
        else { Hand.set(0.0); }
        }
    }
    //------------------------------------------------\\
    /*
    public ColorSensorV3 sensorV3 = new ColorSensorV3(ColorSensor.sensorV3);
    

        // Macro Set:    
        public void Macro (XboxController controller1) throws InterruptedException {
            if (controller1.getTriggerAxis(kLeft) > 0.5) {
                if (controller1.getTriggerAxis(kRight) > 0.5) {
                    if (controller1.getAButtonPressed()) {
                        double spinnerValue = 2.0;
                        Spinner.set(spinnerValue);
                        
                        //if (sensorV3.getRed() = 0 ) {

                        }
                    }
                }   
            }

         */ 

        }
    