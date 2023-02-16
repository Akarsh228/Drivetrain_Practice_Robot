// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.autoMove;
import frc.robot.commands.joyDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private XboxController driveController;
  private final joyDrive jdrive;
  private final  Vision vision;
  private final autoMove autoMove;
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    driveController = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    vision = new Vision();
    jdrive = new joyDrive(drivetrain, driveController, vision);
    autoMove = new autoMove(drivetrain, vision, driveController);

    configureTalons();
    // Drivetrain.EncoderL.setDistancePerPulse(Math.PI* Constants.whd / Constants.cpr );
    // Drivetrain.EncoderR.setDistancePerPulse(Math.PI* Constants.whd / Constants.cpr);
    
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    
  }


  public Command[] getTeleCommand() {
    Command[] ret = {jdrive};
    return ret;
    /* REMINDER: schedule the other commands here !!!! */
  }

  public Command[] getAutoCommand(){
  Command[] ret = {autoMove};
  return ret;
  }
  
  // Auto Commands only once
  public void startAutoInit() {


    // // In 4 feet
    // drivetrain.encoderL.setDistancePerPulse(1.0/256.0);
    // drivetrain.encoderR.setDistancePerPulse(1.0/256.0);
    
  }

  public void startAutoPeriod() {
// moves robot 4 feet backwards

    // if (drivetrain.encoderL.getDistance() <= 4) {

    //   drivetrain.ddrive.arcadeDrive(0.5, 0);
      

    // }
    // else {
    //   // Turns turret to left if at right and right if at left and if it is between
    //   // -1 and 1 it just disables the turret and starts shooting
    //     if (vision.getX() > 1.0) {
    //       turret.turnTurret(-1);
    //     }
    //     else if (vision.getX() < -1.0) {
    //       turret.turnTurret(1);

    //     }
    //     else {
    //       turret.turnTurret(0);
        
          
    //       // At this point in the auto the robot should have the ball in its intake but facing in the wrong direction
    //       // double intakeSpeedRev = shoot.computeV(vision.getY());
          

    //       // Until the intake is empty keep shooting
    //       if (intake.banner1Output()) {
    //         // shooter.outakeV(intakeSpeedRev);
    //         intake.conveyor(0.5);
    //         // Waits until the shooters velocity is within 50 rpm of the vision based speed
    //         // if (shooter.getShooterVel() > (intakeSpeedRev - 50)) {
    //           // intake.transitionMotor(1);
    //         // }
    //       }
    //     }
    // }
  }

  public void configureTalons() {
    drivetrain.l1.setNeutralMode(NeutralMode.Brake);
    drivetrain.l2.setNeutralMode(NeutralMode.Brake);
    drivetrain.r1.setNeutralMode(NeutralMode.Brake);
    drivetrain.r2.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putBoolean("Talons Configured", true);
    SmartDashboard.updateValues();
  }
}

  // public void intakeUp() {
  //   intake.intakeUp();
  // }fd ifd 