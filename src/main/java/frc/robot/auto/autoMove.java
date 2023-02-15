// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.joyDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.limeLight;

public class autoMove extends CommandBase {
  /** Creates a new autoDriveTrain. */
  private final Drivetrain driveAuto;
  private XboxController controllerAuto;
  private limeLight limeLightAuto;
  private double xVisionAuto, yVisionAuto, areaVisionAuto;
  private double encoderLeftDistanceAuto, encoderRightDistanceAuto;
  
  public autoMove (Drivetrain drivetrain, limeLight limeLight, XboxController controller) {
      this.driveAuto = drivetrain;
      this.controllerAuto  = controller;
      this.limeLightAuto = limeLight;

  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encoderLeftDistanceAuto = driveAuto.getLeftEncoderDistance();
    encoderRightDistanceAuto = driveAuto.getRightEncoderDistance();
    smartDashAuto(xVisionAuto, yVisionAuto, areaVisionAuto);

    drivefb(driveAuto, encoderLeftDistanceAuto);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  public void smartDashAuto(double x, double y, double area) {
    SmartDashboard.putNumber("X val of limelight", x);
    SmartDashboard.putNumber("Y val of limelight", y);
    SmartDashboard.putNumber("Area val of limelight", area);
  }

  public void drivefb(Drivetrain drive, double encoderDistance){
    while (encoderDistance < 100){
      drive.move(0.5, 0);
    }

    while (encoderDistance > 0){
      drive.move(-0.5, 0);
    }
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    }
  }
 