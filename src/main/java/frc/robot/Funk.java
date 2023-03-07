package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer; hehe
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Funk {

//DECLARE
int controllerType;
double output;
int output2;
boolean bool;

private final GenericHID stick = new GenericHID(0);


    public void controllerSet(String cont){
        if (cont == "Zorro"){
            controllerType = 0;
        }
        if(cont == "Joystick"){
            controllerType = 1;
        }
    }

    public double controllerAxis(String input){
        if (controllerType == 0){
            if(input == "x1"){output = stick.getRawAxis(0);}
            if(input == "y1"){output = stick.getRawAxis(1);}
            if(input == "pot1"){output = stick.getRawAxis(2);}
            if(input == "x2"){output = stick.getRawAxis(3);}
            if(input == "y2"){output = stick.getRawAxis(4);}
            if(input == "pot2"){output = stick.getRawAxis(5);}
        }
        if (controllerType == 1){
            if(input == "x1"){output = stick.getRawAxis(0);}
            if(input == "y1"){output = stick.getRawAxis(1);}
            if(input == "x2"){output = stick.getRawAxis(2);}
            if(input == "pot1"){output = -stick.getRawAxis(3);}
        }
        return output;
    }

        public int controllerButton(String input){
            if (controllerType == 0){
                if(input == "left1"){bool = stick.getRawButton(6);}
            }
            if (controllerType == 1){

            }
            if (bool = true){
                output2 =1;
            }
            else{
                output2 = 0;
            }
            return output2;
        
    }

    

    public double weightedTurn(double input){
        double weightedTurn = input*(1-(Math.abs(controllerAxis("y1")*0.5)));
        return(weightedTurn);
    }
         
}