package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

//import com.kauailabs.navx.frc.AHRS;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.command.PIDSubsystem;


//import frc.robot.sensors.Limelight;
//import frc.robot.sensors.Pipeline;

public class Arm {
    // INITILIZATION:

    //--------------------------------\\
    // Controller:
    public XboxController controller1 = new XboxController(1); //Really Controller #2 But Base Zero so it is called "1".
    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;     //--------------------------------\\
    // Arm Section:
    public WPI_TalonSRX Arm;    // Will Likely Undergo Change.
    public WPI_TalonSRX Mouth;  // Will Likely Undergo Change.
    //--------------------------------\\
    // Values Defined:
    public double intakeVelocity = 0.0;  // Value to be set after testing.
    public double extakeVelocity = 0.0; // Value to be set after testing.
    public double liftVelocity = 0.0;  // Value to be set after testing.
    public double lowerVelocity = 0.0;// Value to be set after testing.
    //--------------------------------\\
    // Limit Switched Defined:
    DigitalInput lowpointlimit = new DigitalInput(0);
    DigitalInput rightpointlimit = new DigitalInput(1);
    public static DigitalInput digital0;
    public static DigitalInput digital1;
    private DigitalInput limit_ = new DigitalInput(constants.zeroer);

    //------------------------------------------------\\

    // Operator Must Push Trigger, then Right Axis to Control Intake.
        // Failsafe was set since Operator is a Rookie.


    //------------------------------------------------\\
    public void intake() {
        // For controlling the Intake: 
        if (controller1.getTriggerAxis(kRight) > 0.5) {
            if (controller1.getAButtonPressed()){
                Mouth.set(intakeVelocity);
            } 
            // FailSafe (I-1):
            else { Mouth.set(0.0); }
            }

        // Failsafe (I-2):
        if (controller1.getAButtonReleased()) {
            Mouth.set(0.0);
            }
        // Failsafe (I-3):
        if (controller1.getTriggerAxis(kRight) < 0.5) {
            Mouth.set(0.0);
            }
    }

    //------------------------------------------------\\
    public void extake() {
        // For controlling the Extake:
        if (controller1.getTriggerAxis(kRight) > 0.5) {
            if (controller1.getBButtonPressed()) {
                Mouth.set(extakeVelocity);
            }
            // FailSafe (E-1):
            else { Mouth.set(0.0); }
        }
        // Failsafe (E-2):
        if (controller1.getAButtonReleased()) {
            Mouth.set(0.0);
            }
        // Failsafe (E-3):
        if (controller1.getTriggerAxis(kRight) < 0.5) {
            Mouth.set(0.0);
        }
    }
    //------------------------------------------------\\

    // For controlling the Arm (angle):

    //------------------------------------------------\\
    public void lift() {       
        // Raise Up:
        if (controller1.getTriggerAxis(kRight) > 0.5) {
            if (controller1.getYButtonPressed()) {
                Arm.set(liftVelocity);
            } 
            // Failsafe (RU-1):
            else { Arm.set(0.0); }
        }
        // Failsafe (RU-2):
        if (controller1.getYButtonReleased()) {
            Arm.set(0.0);
            }
        // Failsafe (RU-3):
        if (controller1.getTriggerAxis(kRight) < 0.5) {
            Arm.set(0.0);
        }
    }
    //------------------------------------------------\\
    public void lower() {
            // Lower Down:
        if (controller1.getTriggerAxis(kRight) > 0.5) {
            if (controller1.getXButtonPressed()) {
                Arm.set(lowerVelocity);
                //Arm.set(ControlMode.PercentOutput, constants.zeroSpeed);
                if(!limit_.get()) {
                    Arm.set(0.0);
                }
            }
        }
        
            else
            // Failsafe (LD-1):
            { Arm.set(0.0); }
        
        // Failsafe (LD-2):
        if (controller1.getYButtonReleased()) {
            Arm.set(0.0);
            }
        // Failsafe (LD-3):
        if (controller1.getTriggerAxis(kRight) < 0.5) {
            Arm.set(0.0);
        }
    }
    //------------------------------------------------\\
}