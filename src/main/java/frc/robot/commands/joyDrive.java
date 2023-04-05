// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.Date;

import com.revrobotics.ColorMatchResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Color_Sensor;

public class joyDrive extends CommandBase {
  /** Creates a new joyDrive. */
  private final Drivetrain drivetrain;
  private XboxController controller;
  private final Vision vision;
  private final Navx Navx;
  private PIDController pid;
  private Color_Sensor colorSensor;
  private double pidOutput;
  private boolean targetInFocus;
  
  public joyDrive(Drivetrain drivetrain, XboxController controller, Vision vision, Navx gyro, Color_Sensor color_Sensor) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.vision = vision;
    this.Navx = gyro;
    this.colorSensor = colorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetInFocus = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.move(controller.getLeftY(), controller.getRightX());

    SmartDashboard.putBoolean("targetInFocus", targetInFocus);
    Color detectedColor = Color_Sensor.m_colorSensor.getColor();
    ColorMatchResult match = Color_Sensor.m_colorMatcher.matchClosestColor(detectedColor);
    
    if (controller.getBButton()){
      searchForTarget(match, detectedColor, 0.3);
    }

    if(controller.getLeftBumper()){
      BackUp();
    }

    aimBot(vision.getX());
    autoAlign();

    
    SmartDashboard.putNumber("AHRS Nav Angle Yaw", Navx.gyro.getPitch());

    // bellow to check if encoders are working and if distance is right, remove after testing
  }
//change bellow to pid loop.

  public void searchForTarget(ColorMatchResult match, Color detectedColor, double offset){
    while (!colorSensor.getConeDetected(match) && !colorSensor.getCubeDetected(match, detectedColor) ){
      drivetrain.move(0,offset);
    }
    if (colorSensor.getConeDetected(match) || colorSensor.getCubeDetected(match, detectedColor)){
      targetInFocus = true;
    }
  }

  public void BackUp(){
    if (targetInFocus){
    long startTime = System.currentTimeMillis();
    long elapsedTime = 0L;
      
      while (elapsedTime < 2*60*1000) {
          //perform db poll/check
          drivetrain.move(0.3,0);
          elapsedTime = (new Date()).getTime() - startTime;
      }

    }
  }
  public void aimBot (double x) {
    pidOutput = MathUtil.clamp(pid.calculate(x, 0), -.05, 0.5 );

    if (controller.getAButton()) {
      if (vision.getV() == 1.0) {
        SmartDashboard.putString("Working","True");
        SmartDashboard.putNumber("X while Auto Aim", x);
        SmartDashboard.putNumber("PID Output", pidOutput);

        drivetrain.move(0, pidOutput);
        }
      }

    }


  public void autoAlign(){
    if(controller.getXButton()){
      if (Navx.gyro.getPitch() >= 7) {
        drivetrain.move(.45, 0);
      }

      else if (Navx.gyro.getPitch()<= -7) {
        drivetrain.move(-.45, 0);
      }

      else if((Navx.gyro.getPitch() >= 4) && (Navx.gyro.getPitch() < 7)){
        drivetrain.move(.35, 0);
      }
      else if((Navx.gyro.getPitch() <= -4) && (Navx.gyro.getPitch() > -7)){
        drivetrain.move(-.35, 0);
      }
    
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
