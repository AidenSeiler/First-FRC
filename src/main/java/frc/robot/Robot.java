package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//mport edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;



public class Robot extends TimedRobot {
  PowerDistribution powerPanel = new PowerDistribution(1, ModuleType.kRev);
  private Funk config = new Funk();
  private WPI_VictorSPX motor1;
  private WPI_VictorSPX motor2; 
  private WPI_VictorSPX motor3;
  private WPI_VictorSPX motor4;
  private DifferentialDrive robotDrive;
  Spark blinkin = new Spark(0);
  //private Compressor blowJob = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid testSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);  

  @Override

  public void robotInit() {
    motor1 = new WPI_VictorSPX(1);
    motor2 = new WPI_VictorSPX(2);
    motor3 = new WPI_VictorSPX(3);
    motor4 = new WPI_VictorSPX(4);
    MotorControllerGroup left = new MotorControllerGroup(motor1, motor2);
    MotorControllerGroup right = new MotorControllerGroup(motor3, motor4);
   robotDrive = new DifferentialDrive(left, right);
   CameraServer.startAutomaticCapture();
  config.controllerSet("Zorro");
  }@Override

  //public void autonomousInit() {}@Override
  //public void autonomousPeriodic() {}@Override


  public void teleopInit() {}
   double totalCurrent;
   double turn;
   double throttle;
   int button = 0;
   
  @Override


  public void teleopPeriodic() {



   // turn = config.weightedTurn(config.controllerAxis("x2"));
    turn = config.controllerAxis("x2");
    throttle = config.controllerAxis("y1");
    robotDrive.arcadeDrive(turn,-  throttle);
    blinkin.set(0.53);

    if (config.controllerButton("twoWay1") == 0){
      testSolenoid.set(DoubleSolenoid.Value.kForward);}
      else if (config.controllerButton("twoWay1") == 1){
      testSolenoid.set(DoubleSolenoid.Value.kReverse);}
      else {
      testSolenoid.set(DoubleSolenoid.Value.kOff);
      }
    



    SmartDashboard.putNumber("Turn Input",config.controllerAxis("x2"));
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Throttle Input",config.controllerAxis("y1"));

    SmartDashboard.putNumber("Total Current", powerPanel.getTotalCurrent());
    SmartDashboard.putNumber("pot1",config.controllerAxis("pot1"));
    SmartDashboard.putNumber("pot2",config.controllerAxis("pot2"));

    SmartDashboard.putNumber("threeWay1",config.controllerButton("threeWay1"));
    SmartDashboard.putNumber("threeWay2",config.controllerButton("threeWay2"));
    SmartDashboard.putNumber("twoWay1",config.controllerButton("twoWay1"));
    SmartDashboard.putNumber("twoWay2",config.controllerButton("twoWay2"));
    SmartDashboard.putNumber("topB1",config.controllerButton("topB1"));
    SmartDashboard.putNumber("topB2",config.controllerButton("topB2"));
    SmartDashboard.putNumber("backB1",config.controllerButton("backB1"));
    SmartDashboard.putNumber("backB2",config.controllerButton("backB2"));


   
      

  }@Override
    public void testInit() {}@Override
    public void testPeriodic() {}
}
