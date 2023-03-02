// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private static final int leftMotorID = 1; 
  private static final int rightMotorID = 2;
 
 // private final XboxController m_controller = new XboxController(0);
  private DifferentialDrive robotDrive;
  Joystick stick = new Joystick(0);

  @Override
  public void robotInit() {
   // rightMotor.setInverted(true);
    leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushed);
    rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushed);
    robotDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    robotDrive.arcadeDrive(-stick.getZ()*(-stick.getThrottle()+1.5)/2, stick.getY()*(-stick.getThrottle()+1.5)/2);
   // robotDrive.arcadeDrive(-m_controller.getRightX()*0.75,m_controller.getLeftY());

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
