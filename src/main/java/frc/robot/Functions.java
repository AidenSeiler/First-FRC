package frc.robot;

//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer; hehe
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Functions {

    Joystick stick = new Joystick(0);

    public double turningCurve(double input){
        double finalTurn = input*(1-(Math.abs(stick.getY()*0.5)));
        return(finalTurn);
    }
         
}