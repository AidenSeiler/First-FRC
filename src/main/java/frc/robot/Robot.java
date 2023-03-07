package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer; hehe
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj.Compressor;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
>>>>>>> Stashed changes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Robot extends TimedRobot {
<<<<<<< Updated upstream
  //private PowerDistributionPanel examplePDP = new PowerDistributionPanel(0);
=======
  PowerDistribution powerPanel = new PowerDistribution(1, ModuleType.kRev);
  private Funk config = new Funk();
  private WPI_VictorSPX motor1;
  private WPI_VictorSPX motor2; 
  private WPI_VictorSPX motor3;
  private WPI_VictorSPX motor4;
  private DifferentialDrive robotDrive;
  Spark blinkin = new Spark(0);
  private Compressor blowJob = new Compressor(1, PneumaticsModuleType.REVPH);

>>>>>>> Stashed changes

  private Functions config = new Functions();
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private static final int leftMotorID = 1; 
  private static final int rightMotorID = 2;
  private DifferentialDrive robotDrive;
  Joystick stick = new Joystick(0);

  @Override

  public void robotInit() {
<<<<<<< Updated upstream
    leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
    rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);
    robotDrive = new DifferentialDrive(leftMotor, rightMotor);
=======
    motor1 = new WPI_VictorSPX(1);
    motor2 = new WPI_VictorSPX(2);
    motor3 = new WPI_VictorSPX(3);
    motor4 = new WPI_VictorSPX(4);
    MotorControllerGroup left = new MotorControllerGroup(motor1, motor2);
    MotorControllerGroup right = new MotorControllerGroup(motor3, motor4);

<<<<<<< HEAD
   robotDrive = new DifferentialDrive(left, right);
  // CameraServer.startAutomaticCapture();
=======
   robotDrive = new DifferentialDrive(motor1, motor2);
   CameraServer.startAutomaticCapture();
>>>>>>> 679a62cccbbd751e3eee28553902544656895813
  config.controllerSet("Zorro");

>>>>>>> Stashed changes
  }@Override

  //public void autonomousInit() {}@Override
  //public void autonomousPeriodic() {}@Override


  public void teleopInit() {}
  double throttle;
  double turn;

  @Override


  public void teleopPeriodic() {
  throttle = stick.getY();
  turn = -config.turningCurve(stick.getZ());

<<<<<<< Updated upstream
  robotDrive.arcadeDrive(turn,throttle);
=======
    throttle = config.controllerAxis("y1");
    robotDrive.arcadeDrive(turn,-  throttle);
    blinkin.set(0.53);
    SmartDashboard.putNumber("Turn Input",config.controllerAxis("x2"));
    SmartDashboard.putNumber("Pot",config.controllerAxis("pot1"));
    SmartDashboard.putNumber("leftTopButton",config.controllerButton("leftB1"));
    SmartDashboard.putNumber("Throttle Input",config.controllerAxis("y1"));
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Total Current", powerPanel.getTotalCurrent());
>>>>>>> Stashed changes

  SmartDashboard.putNumber("Turn Input",stick.getZ() );
  SmartDashboard.putNumber("Turn Output", turn);

<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
  }@Override
  public void testInit() {}@Override
  public void testPeriodic() {}
}
