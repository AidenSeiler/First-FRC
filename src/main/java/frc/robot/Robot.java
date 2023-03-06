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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;



public class Robot extends TimedRobot {
  PowerDistribution powerPanel = new PowerDistribution(1, ModuleType.kRev);
  private Funk config = new Funk();
  //private CANSparkMax leftMotor;
  //private CANSparkMax rightMotor;
  private DifferentialDrive robotDrive;
  Spark blinkin = new Spark(0);
  private Compressor blowJob = new Compressor(1, PneumaticsModuleType.REVPH);
  private speedcontroller leftMotor = new WPI_VictorSPX(0);
  private speedcontroller rightMotor = new WPI_VictorSPX(0);


  @Override

  public void robotInit() {
    private speedcontroller leftMotor = new WPI_VictorSPX(0);

   leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
   rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);
   robotDrive = new DifferentialDrive(leftMotor, rightMotor);
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
    turn = config.weightedTurn(config.controllerAxis("x2"));
    throttle = config.controllerAxis("y1");
    robotDrive.arcadeDrive(-turn,-  throttle);
    blinkin.set(0.53);
    SmartDashboard.putNumber("Turn Input",config.controllerAxis("x2"));
    SmartDashboard.putNumber("Throttle Input",config.controllerAxis("y1"));
    SmartDashboard.putNumber("Turn Output",turn);
    SmartDashboard.putNumber("Total Current", powerPanel.getTotalCurrent());

  }@Override
    public void testInit() {}@Override
    public void testPeriodic() {}
}
