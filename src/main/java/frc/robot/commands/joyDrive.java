// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Navx;

public class joyDrive extends CommandBase {
  /** Creates a new joyDrive. */
  private final Drivetrain drivetrain;
  private XboxController controller;
  private final Vision vision;
  private final Navx Navx;
  private PIDController pid;
  private double pidOutput;
  public joyDrive(Drivetrain drivetrain, XboxController controller, Vision vision, Navx gyro) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.vision = vision;
    this.Navx = gyro;
    pid = new PIDController(2, 0, 0);
    pid.setIntegratorRange(-0.1, 0.1);
    pid.setTolerance(.1);
    pid.enableContinuousInput(-0.3, 0.3);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("AHRS Nav Angle Yaw", Navx.gyro.getPitch());
    drivetrain.move(controller.getLeftY(), controller.getRightX());
    aimBot(vision.getX());
    autoAlign();

    // bellow to check if encoders are working and if distance is right, remove after testing
  }
//change bellow to pid loop.
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
