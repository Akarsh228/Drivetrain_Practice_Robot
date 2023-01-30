// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.Math;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public WPI_TalonSRX l1, l2, r1, r2;
  public MotorControllerGroup l, r;
  public DifferentialDrive ddrive;
  public static Encoder EncoderL;
  public static Encoder EncoderR;

  public Drivetrain ()
  {
    l1 = new WPI_TalonSRX(Constants.MOTOR_L1_ID);
    l2 = new WPI_TalonSRX(Constants.MOTOR_L2_ID);
    r1 = new WPI_TalonSRX(Constants.MOTOR_R1_ID);
    r2 = new WPI_TalonSRX(Constants.MOTOR_R2_ID);
  
    r1.setInverted(true);
    r2.setInverted(true);

    l2.follow(l1);
    r2.follow(r1); 
    EncoderR.setReverseDirection(true);

    l = new MotorControllerGroup(l1, l2);
    r = new MotorControllerGroup(r1, r2);
    ddrive = new DifferentialDrive(l,r);


    EncoderL  = new Encoder(Constants.L_ENCODER_CHANNEL_A, Constants.L_ENCODER_CHANNEL_B);   
    EncoderR = new Encoder(Constants.R_ENCODER_CHANNEL_A, Constants.R_ENCODER_CHANNEL_B); 
   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void move (double power, double offset){
    ddrive.arcadeDrive(power,offset);
  }

  public double getEncoderDistance(){
    SmartDashboard.putNumber("ENCODER_L", EncoderL.getDistance());
    SmartDashboard.updateValues();

    
    return EncoderL.getDistance();
  }

}
/* 
@Override
public void autonomousInit() {
  auto = m_robotContainer.getAutoCommand();
  auto.schedule();
}
*/