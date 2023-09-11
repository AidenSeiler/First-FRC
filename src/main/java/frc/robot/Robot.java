package frc.robot;
//import robot
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import Vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//importPID
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import Camera Mask
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
public class Robot extends TimedRobot {

NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);
  
  
Thread m_visionThread;

Funk config = new Funk();
//CAN DEVICES
  PowerDistribution powerPanel = new PowerDistribution(1, ModuleType.kRev);
  Compressor comp = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid testSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);  

//MOTORS
  CANSparkMax right1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax left1 = new CANSparkMax(2 , MotorType.kBrushless);

  WPI_VictorSPX right2 = new WPI_VictorSPX(1);
  WPI_VictorSPX left2 = new WPI_VictorSPX(2);

  SparkMaxPIDController leftDrivePID;
  RelativeEncoder leftEncoder;
  SparkMaxPIDController rightDrivePID;
  RelativeEncoder rightEncoder;
  SparkMaxPIDController tiltPID;
  RelativeEncoder tiltEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  
//VARIABLES
  double totalCurrent;
  double turn;
  double throttle;
  int button = 0;
  double power = 1;
  double maxCurrent = 100;

  

//VISION_STOFF
  // int imageHeight = 480;
  // int imageWidth = 640;  
  // double xOffset;
  // double xRange;

  //PID
  PIDController pid = new PIDController(1, 0, 0);
  VisionThread visionThread;

  LinearFilter xfilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  LinearFilter yfilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  double rawObjectY;
double xFinal;
double yFinal;
double rawObjectX;


  @Override
  public void robotInit() {

  //   UsbCamera camera = CameraServer.startAutomaticCapture();
  //   camera.setResolution(imageWidth, imageHeight);
  //  Object imgLock = new Object();
  //   visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
  //     if (!pipeline.filterContoursOutput().isEmpty()) {
  //         Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
  //         synchronized (imgLock) {
  //              rawObjectX = r.x + (r.width / 2);
  //              rawObjectY = r.y + (r.height / 2);

  //         }
  //     }
  // });
  // visionThread.start();

  m_visionThread =
  new Thread(
      () -> {
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          Imgproc.circle(
              mat, new Point(rawObjectX, rawObjectY), 10, new Scalar(0, 255, 0), 5);

          // Give the output stream a new image to display

          outputStream.putFrame(mat);

        }

      });

m_visionThread.setDaemon(true);

m_visionThread.start();

//PID INIT
    leftDrivePID = left1.getPIDController();
    leftEncoder = left1.getEncoder();
    rightDrivePID = right1.getPIDController();
    rightEncoder = right1.getEncoder();

   // DRIVE PID coefficients
    kP = 0.00006; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 6000;

    leftDrivePID.setP(kP);
    leftDrivePID.setI(kI);
    leftDrivePID.setD(kD);
    leftDrivePID.setIZone(kIz);
    leftDrivePID.setFF(kFF);
    leftDrivePID.setOutputRange(kMinOutput, kMaxOutput);


    rightDrivePID.setP(kP);
    rightDrivePID.setI(kI);
    rightDrivePID.setD(kD);
    rightDrivePID.setIZone(kIz);
    rightDrivePID.setFF(kFF);
    rightDrivePID.setOutputRange(kMinOutput, kMaxOutput);



   
  }@Override
  //public void autonomousInit() {}@Override
  //public void autonomousPeriodic() {}@Override


  public void teleopInit() {
    //CAMERA SETTINGS
   

    //CONTROLLER SELECTION
    config.controllerSet("Zorro");
  }@Override


  public void teleopPeriodic() {
   // xOffset = (rawObjectX - (imageWidth/2))/(imageWidth/2);
    totalCurrent = powerPanel.getTotalCurrent();



if(config.controllerButton("topB2") == 1){
 if(true){

 }
}
    //DROBING
    turn = config.deadzone(config.controllerAxis("x1"))+xFinal;
    throttle = -config.controllerAxis("y2");
    double setPointLeft = (-throttle+turn)*maxRPM;
   leftDrivePID.setReference(setPointLeft, CANSparkMax.ControlType.kVelocity);
    double setPointRight = (throttle+turn)*maxRPM;
    rightDrivePID.setReference(setPointRight, CANSparkMax.ControlType.kVelocity);
    //left2.set(-throttle+turn);
    //right2.set(throttle+turn);

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
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Total Current", totalCurrent);
    SmartDashboard.putNumber("pot1",config.controllerAxis("pot1"));
    SmartDashboard.putNumber("pot2",config.controllerAxis("pot2"));
   // SmartDashboard.putNumber("Normalized X",xOffset);
    SmartDashboard.putNumber("Final X Adjustment",xFinal);
    SmartDashboard.putNumber("Object Raw Y",rawObjectY);
    SmartDashboard.putNumber("Object Raw X",rawObjectX);
    // SmartDashboard.putNumber("threeWay1",config.controllerButton("threeWay1"));
    // SmartDashboard.putNumber("threeWay2",config.controllerButton("threeWay2"));
    // SmartDashboard.putNumber("twoWay1",config.controllerButton("twoWay1"));
    // SmartDashboard.putNumber("twoWay2",config.controllerButton("twoWay2"));
    // SmartDashboard.putNumber("topB1",config.controllerButton("topB1"));
    // SmartDashboard.putNumber("topB2",config.controllerButton("topB2"));
    // SmartDashboard.putNumber("backB1",config.controllerButton("backB1"));
    // SmartDashboard.putNumber("backB2",config.controllerButton("backB2"));
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // if((p != kP)) { leftDrivePID.setP(p);rightDrivePID.setP(p); kP = p; }
    // if((i != kI)) { leftDrivePID.setI(i);rightDrivePID.setI(i); kI = i; }
    // if((d != kD)) { leftDrivePID.setD(d);rightDrivePID.setD(d); kD = d; }
    // if((iz != kIz)) { leftDrivePID.setIZone(iz);rightDrivePID.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { leftDrivePID.setFF(ff);rightDrivePID.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   leftDrivePID.setOutputRange(min, max);rightDrivePID.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }@Override
    public void testInit() {}@Override
    public void testPeriodic() {

    }
}

