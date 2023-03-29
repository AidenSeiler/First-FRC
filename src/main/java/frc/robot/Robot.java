package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Robot extends TimedRobot {
  Funk config = new Funk();

//CAN DEVICES
  PowerDistribution powerPanel = new PowerDistribution(1, ModuleType.kRev);
  Compressor comp = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid testSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);  

//MOTORS
  CANSparkMax spinner1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax spinner2 = new CANSparkMax(2, MotorType.kBrushless);
  WPI_VictorSPX motor1 = new WPI_VictorSPX(1);
  WPI_VictorSPX motor2 = new WPI_VictorSPX(2);
  WPI_VictorSPX motor3 = new WPI_VictorSPX(3);
  WPI_VictorSPX motor4 = new WPI_VictorSPX(4);
  MotorControllerGroup left = new MotorControllerGroup(motor1, motor2);
  MotorControllerGroup right = new MotorControllerGroup(motor3, motor4);
  DifferentialDrive robotDrive = new DifferentialDrive(left, right);

//VARIABLES
  double totalCurrent;
  double turn;
  double throttle;
  int button = 0;
  double power = 1;
  double maxCurrent = 100;

//VISION_STOFF
  UsbCamera camera;
  int imageHeight = 480;
  int imageWidth = 640;  
  double xOffset;
  double yOffset;
  double xPIDout;

  //PID
  PIDController pid = new PIDController(0, 0, 0);
  @Override
  //public void robotInit() {}@Override
  //public void autonomousInit() {}@Override
  //public void autonomousPeriodic() {}@Override


  public void teleopInit() {
    //CAMERA SETTINGS
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(imageWidth,imageHeight);
    camera.setFPS(30);

    //CONTROLLER SELECTION
    config.controllerSet("Zorro");
  }@Override


  public void teleopPeriodic() {
    comp.disable();
    totalCurrent = powerPanel.getTotalCurrent();
    //NETWORK TABLE
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
    NetworkTableEntry xData = table.getEntry("COG_X");
    NetworkTableEntry yData = table.getEntry("COG_Y");
    inst.startClientTeam(1234); 
    inst.startDSClient(); 
    double x = xData.getDouble(0.0);
    double y = yData.getDouble(0.0);

    xOffset = (x-(imageWidth/2))/(imageWidth/2);
    yOffset = (y-(imageHeight/2))/(imageHeight/2);
    xPIDout = MathUtil.clamp(pid.calculate(xOffset, 0), -0.1, 0.1);

    turn = config.weightedTurn(config.controllerAxis("x2"))+xPIDout;
    throttle = config.controllerAxis("y2");
    robotDrive.arcadeDrive(turn,-  throttle);
//SPINNER ENABLE AND DISABLE/SPEED CONTROL
    if(config.controllerButton("topB1") == 1){
      spinner1.set(config.controllerAxis("pot1"));
      spinner2.set(-config.controllerAxis("pot2"));
    }
    else{
      spinner1.set(0);
      spinner2.set(0);
    }
//SOLENOID ACTUATION
    if (config.controllerButton("twoWay1") == 1){
      testSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      testSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
//COMPRESSOR ON/OFF
    if (config.controllerButton("twoWay2") == 1){
      comp.disable();
    } 
    else{
      comp.enableDigital();
    }


   
//DASHBOARD
    SmartDashboard.putNumber("Turn Input",config.controllerAxis("x2"));
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Throttle Input",config.controllerAxis("y1"));
    SmartDashboard.putNumber("Total Current", totalCurrent);
    SmartDashboard.putNumber("pot1",config.controllerAxis("pot1"));
    SmartDashboard.putNumber("pot2",config.controllerAxis("pot2"));
    SmartDashboard.putNumber("Image x",x);
    SmartDashboard.putNumber("Image y",y);
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

