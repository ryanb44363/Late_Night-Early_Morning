
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

public class Climb {
    
    //--------------------------------\\
    // Climb Section:
    public WPI_TalonSRX Hook;       // Will Likely Undergo Change.
    public WPI_TalonSRX Climb_Motor1;// Will Likely Undergo Change.
    public WPI_TalonSRX Climb_Motor2;// Will Likely Undergo Change.
    //--------------------------------\\
    // Controller:
    public XboxController controller1 = new XboxController(1); //Really Controller #2 But Base Zero so it is called "1".
    public static final XboxController.Hand kLeft = XboxController.Hand.kLeft;
    public static final XboxController.Hand kRight = XboxController.Hand.kRight;
    //--------------------------------\\
    //--------------------------------\\
    // Variables:
    double upperVelocity = 0.0; // Value to be set after testing.
    double lowerVelocity = 0.0; // Value to be set after testing.
    double climbUpVelocity = 0.25; // Value to be set after testing.
    //--------------------------------\\
    
    //------------------------------------------------\\
    public void hoist() {
            
            // Hoist Hook:
            if (controller1.getTriggerAxis(kRight) > 0.5) {
                if (controller1.getY(kLeft) > 0.05) {
                    Hook.set(upperVelocity);
                }
                if (controller1.getY(kLeft) < 0.05) {
                    Hook.set(lowerVelocity);
                }
            }
            // Failsafe (H-1):
            else
                { Hook.set(0.0); }
            // Failsafe (H-2):
            if (controller1.getTriggerAxis(kRight) < 0.05 || controller1.getTriggerAxis(kRight) > 0.05) {
                Hook.set(0.0);
                }
        }
        //------------------------------------------------\\
    public void climb() {
            // Climb Up:
            if (controller1.getTriggerAxis(kRight) > 0.05) {
                if (controller1.getY(kRight) > 0.05) {
                    Climb_Motor1.set(climbUpVelocity);  // "throttle = controller.getY(kLeft);"  Keep this in mind!!!
                    Climb_Motor2.set(climbUpVelocity);  
                } 
            // Failsafe (C-1):
            else {
                Climb_Motor1.set(0.0);
                Climb_Motor2.set(0.0);
                }
            }
            // Failsafe (C-2):
            if (controller1.getTriggerAxis(kRight) < 0.5) {
                Climb_Motor1.set(0.0);
                Climb_Motor2.set(0.0);
            }
        }
        //------------------------------------------------\\
    }
