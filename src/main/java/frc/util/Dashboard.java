
package frc.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edi.wpi.first.wpilibj2.command.Shuffleboard;
public class Dashboard() {

  Shuffleboard.get(BatteryPercent());

  public void BatteryPercent() {
    double[] voltages = new double[20];
    double sum = 0;
    for(int i = 0;;) {
      voltages[i] = m_pdp.getVoltage();
      if (i < 20) {i++;} else {i = 0;}
      wait(500);
    }
    for (int i = 0; i < voltages.length; i++) {
      sum += voltages[i];
    }   
    double averageVoltage = sum/voltages.length;
    sum = 0;
    double batteryEstimate = (averageVoltage - 10)/3.6;
  }
}