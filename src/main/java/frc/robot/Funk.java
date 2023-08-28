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
boolean out;



final GenericHID stick = new GenericHID(0);


    public void controllerSet(String cont){
        if (cont == "Zorro"){
            controllerType = 0;
        }
        if(cont == "Joystick"){
            controllerType = 1;
        }
    }

    public double deadzone(double input){
        if (Math.abs(input) > 0.0){
            return input;
        }
        else{
        return 0;
        }
    }



    public double controllerAxis(String input){
        if (controllerType == 0){//Zorro
            if(input == "x1"){output = stick.getRawAxis(0);}
            if(input == "y1"){output = stick.getRawAxis(1);}
            if(input == "pot1"){output = stick.getRawAxis(2);}
            if(input == "x2"){output = stick.getRawAxis(3);}
            if(input == "y2"){output = stick.getRawAxis(4);}
            if(input == "pot2"){output = stick.getRawAxis(5);}
        }
        if (controllerType == 1){//Joystick
            if(input == "x2"){output = stick.getRawAxis(2);}
            if(input == "y2"){output = -stick.getRawAxis(1);}
            if(input == "x1"){output = stick.getRawAxis(0);}
            if(input == "pot1"){output = -stick.getRawAxis(3);}
        }
        
        return output;
      
    }

//Zorro
    public int controllerButton(String input){
        if (controllerType == 0){
            if((input == "threeWay1") && stick.getRawAxis(6) == 1){output2 = 0;}
            if((input == "threeWay1") && stick.getRawAxis(6) == 0){output2 = 1;}
            if((input == "threeWay1") && stick.getRawAxis(6) == -1){output2 = 2;}
            if((input == "threeWay2") && stick.getRawButton(1)){output2 = 0;}
            if((input == "threeWay2") && stick.getRawButton(2)){output2 = 1;}
            if((input == "threeWay2") && stick.getRawButton(3)){output2 = 2;}
            if((input == "twoWay1") && stick.getRawButton(4)){output2 = 1;}
            if((input == "twoWay1") && !stick.getRawButton(4)){output2 = 0;}
            if((input == "twoWay2") && stick.getRawButton(5)){output2 = 1;}
            if((input == "twoWay2") && !stick.getRawButton(5)){output2 = 0;}
            if((input == "topB1") && stick.getRawButton(6)){output2 = 1;}
            if((input == "topB1") && !stick.getRawButton(6)){output2 = 0;}
            if((input == "topB2") && stick.getRawButton(7)){output2 = 1;}
            if((input == "topB2") && !stick.getRawButton(7)){output2 = 0;}   
            if((input == "backB1") && stick.getRawButton(8)){output2 = 1;}
            if((input == "backB1") && !stick.getRawButton(8)){output2 = 0;}  
            if((input == "backB2") && stick.getRawButton(9)){output2 = 1;}
            if((input == "backB2") && !stick.getRawButton(9)){output2 = 0;}   
        }
        //Joystick
        if(controllerType ==1){
            if((input == "threeWay1") && stick.getRawButton(2)){output2 = 1;}
            if((input == "threeWay1") && !stick.getRawButton(2)){output2 = 0;}
            if((input == "twoWay1") && stick.getRawButton(1)){output2 = 0;}
            if((input == "twoWay1") && !stick.getRawButton(1)){output2 = 1;}
            if((input == "topB2") && stick.getRawButton(2)){output2 = 1;}
            if((input == "topB2") && !stick.getRawButton(2)){output2 = 0;}
            if((input == "twoWay2") && stick.getRawButton(3)){output2 = 0;}
            if((input == "twoWay2") && !stick.getRawButton(3)){output2 = 1;}
            if((input == "backB1") && stick.getRawButton(5)){output2 = 1;}
            if((input == "backB1") && !stick.getRawButton(5)){output2 = 0;} 
        }
       
        return output2;
    }

    public double weightedTurn(double input){
        double weightedTurn = input*(1-(Math.abs(controllerAxis("y1")*0.5)));
        return(weightedTurn);
    }
         
}