// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.limeLight;

public class vision extends CommandBase {
  private final Drivetrain drivetrain;
  private limeLight limeLightV;
  private XboxController controller;
  private double xVision, yVision, areaVision;
  
  public vision (Drivetrain drivetrain, limeLight limelight, XboxController controller) {
    this.drivetrain = drivetrain;
    this.limeLightV = limelight;
    this.controller=controller;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xVision = limeLightV.getVisionTX();
    yVision = limeLightV.getVisionTY();
    areaVision = limeLightV.getVisionTA();

    autoAim(xVision, yVision, areaVision, controller, drivetrain);
    smartDashVision(xVision, yVision, areaVision);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public void autoAim (double x,  double y, double area, XboxController controller, Drivetrain drive) {
    while (controller.getAButton() == true) {
      if (area > 0) {
        if (x<0){
          drive.move(0,.3);
        }

        else if (x>0){
          drive.move(0,-.3);
        }
      }
      else {
        continue;
      }
    }
  }

  public void smartDashVision(double x, double y, double area) {
    SmartDashboard.putNumber("X val of limelight", x);
    SmartDashboard.putNumber("Y val of limelight", y);
    SmartDashboard.putNumber("Area val of limelight", area);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
