package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
  private Compressor blowJob = new Compressor(1, PneumaticsModuleType.REVPH);



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

  @Override


  public void teleopPeriodic() {
   // turn = config.weightedTurn(config.controllerAxis("x2"));
    turn = config.controllerAxis("x2");

    throttle = config.controllerAxis("y1");
    robotDrive.arcadeDrive(turn,-  throttle);
    blinkin.set(0.53);
    SmartDashboard.putNumber("Turn Input",config.controllerAxis("x2"));
    SmartDashboard.putNumber("Throttle Input",config.controllerAxis("y1"));
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Total Current", powerPanel.getTotalCurrent());

  }@Override
    public void testInit() {}@Override
    public void testPeriodic() {}
}
