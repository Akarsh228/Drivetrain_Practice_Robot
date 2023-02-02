// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public WPI_TalonSRX l1, l2, r1, r2;
  public MotorControllerGroup l, r;
  public DifferentialDrive ddrive;
  //double leftEnocoderValues, rightEncoderValues;
  private double leftEncoderDistance;
  private double rightEncoderDistance;
  
  // public static Encoder EncoderL;
  //public static Encoder EncoderR;


  public Drivetrain ()
  {
    l1 = new WPI_TalonSRX(Constants.MOTOR_L1_ID);
    l2 = new WPI_TalonSRX(Constants.MOTOR_L2_ID);
    r1 = new WPI_TalonSRX(Constants.MOTOR_R1_ID);
    r2 = new WPI_TalonSRX(Constants.MOTOR_R2_ID);

    leftEncoderDistance = 0;
    rightEncoderDistance = 0;

    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);

    leftEncoderDistance = (Math.PI * Constants.whd/ Constants.cpr) * l1.getSelectedSensorPosition();
    rightEncoderDistance = (Math.PI * Constants.whd/ Constants.cpr) * r1.getSelectedSensorPosition();

    l1.setInverted(true);
    l2.setInverted(true);

    //Drivetrain.EncoderL.setDistancePerPulse(Math.PI* Constants.whd / Constants.cpr );
    //Drivetrain.EncoderR.setDistancePerPulse(Math.PI* Constants.whd / Constants.cpr);
    
    l2.follow(l1);
    r2.follow(r1); 
    //EncoderR.setReverseDirection(true);

    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    ddrive = new DifferentialDrive(l,r);


    //EncoderL  = new Encoder(Constants.L_ENCODER_CHANNEL_A, Constants.L_ENCODER_CHANNEL_B);   
    // EncoderR = new Encoder(Constants.R_ENCODER_CHANNEL_A, Constants.R_ENCODER_CHANNEL_B); 
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void move (double power, double offset){
    ddrive.arcadeDrive(power,offset);
  }

  public double getLeftEncoderSensorValue(){
    SmartDashboard.putNumber("ENCODER_L_SENSOR_VALUES", l1.getSelectedSensorPosition());
    SmartDashboard.updateValues();
    

    
     return l1.getSelectedSensorPosition();
  }


  public double getRightEncoderSensorValue(){
    SmartDashboard.putNumber("ENCODER_R_SENSOR_VALUES", r1.getSelectedSensorPosition());
    SmartDashboard.updateValues();

    
     return l1.getSelectedSensorPosition();
  }

  public double getLeftEncoderDistance (){
    SmartDashboard.putNumber("ENCODER_L_DISTANCE", leftEncoderDistance);
    return leftEncoderDistance;
  }

  public double getRightEncoderDistance (){
    SmartDashboard.putNumber("ENCODER_R_DISTANCE", rightEncoderDistance);
    return rightEncoderDistance;
  }
}

 
  //double checking if it is working


/* 
@Override
public void autonomousInit() {
  auto = m_robotContainer.getAutoCommand();
  auto.schedule();
}
*/