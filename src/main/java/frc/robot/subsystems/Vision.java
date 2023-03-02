package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public double x;
  NetworkTable table;
  NetworkTableEntry tx, tv;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");

    SmartDashboard.putNumber("x of vision", getX());
    SmartDashboard.updateValues();
  }

  public double getX() {

    x = tx.getDouble(0);

    if (tv.getBoolean(false)) {
      x = 0;

    }
    return x;
  }

  public double getV() {
    return tv.getDouble(3);
  }
}
