// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Color_Sensor extends SubsystemBase {
  /** Creates a new Color_Sensor. */
  public final static ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.i2cPort);
  double IR = m_colorSensor.getIR();
  public final static ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kYellowTarget = new Color(.255, .255, 0);
  private final Color kPurpleTarget = new Color(.141,.52, .135);
  
  public Color_Sensor() {
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kPurpleTarget);
  } 


  @Override
  public void periodic() {
  
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    double IR = m_colorSensor.getIR();
    SmartDashboard.putNumber("Red", Math.round(detectedColor.red * 10));
    SmartDashboard.putNumber("Green", Math.round(detectedColor.green * 10));
    SmartDashboard.putNumber("Blue", Math.round(detectedColor.blue * 10));
    SmartDashboard.putNumber("IR", IR);

    Double rgb = (double) ((100 * Math.round(detectedColor.red * 10)) + (10 * Math.round(detectedColor.red * 10)) +  Math.round(detectedColor.red * 10));

    int proximity = m_colorSensor.getProximity();
    

    boolean cone = (rgb == 222);
    boolean cube =(rgb == 333 || rgb == 444);
    SmartDashboard.putNumber("proximity", proximity );
    SmartDashboard.putNumber("RGB", rgb);
    
    SmartDashboard.putBoolean("Cone", match.color == kYellowTarget); 
    SmartDashboard.putBoolean("Cube", match.color == kPurpleTarget  && Math.round(detectedColor.green * 10) == 4);
    }

    public boolean getConeDetected (ColorMatchResult match){

      if (match.color == kYellowTarget){
        return true;
      }

      else {
        return false;
      }

    }

    public boolean getCubeDetected (ColorMatchResult match, Color detectedColor){

      if (match.color == kPurpleTarget  && Math.round(detectedColor.green * 10) == 4){
        return true;
      }

      else {
        return false;
      }

    }
    
  }