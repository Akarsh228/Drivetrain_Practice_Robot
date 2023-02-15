package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class limeLight extends SubsystemBase {
  public double x;
  public double y;
  public double area;
  NetworkTable table;
  private XboxController controller;

  /** Creates a new limeLighrt. */
  public limeLight(XboxController controller) {

    table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    this.controller = controller;


    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);  

  }

  @Override
  public void periodic() {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0); 

  }

  public double getVisionTX(){
    return x;
  }

  public double getVisionTY(){
    return y;
  }

  public double getVisionTA(){
    return area;
  }


  public void followReflectiveTape(Drivetrain drivetrain){
    
      if(area != 0.0){
       if (x<3){
        while (x<0){
          drivetrain.move(0,.25);
          // bellow is for autonomous just for smoothing
          // if (x<=-2 && x>=2) {break;} 
        } 
       }

       else if (x>0){
        while (x>0){
          drivetrain.move(0,-.25);
          // if (x<=-2 && x>=2) {break;}
        } 
       }
      }
    
  }
}

