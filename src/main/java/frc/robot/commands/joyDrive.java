// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class joyDrive extends CommandBase {
  /** Creates a new joyDrive. */
  private final Drivetrain drivetrain;
  private XboxController controller;
  private final Vision vision;
  
  public joyDrive(Drivetrain drivetrain, XboxController controller, Vision vision) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.move(controller.getLeftY(), controller.getRightX());
    autoAim(vision.getX());

    // bellow to check if encoders are working and if distance is right, remove after testing
  }

  public void autoAim (double x) {
    x = (int) x;
    if (controller.getAButton()) {
      if (vision.getV() == 1.0) {
        SmartDashboard.putString("Working","True");
        SmartDashboard.putNumber("X while Auto Aim", x);
        SmartDashboard.updateValues();
        if (x<-0.5){
          drivetrain.move(0,-.5);
        }

        else if (x>0){
          drivetrain.move(0,.5);
        }
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
