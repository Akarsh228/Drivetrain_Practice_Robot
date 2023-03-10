// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
  /** Creates a new gyro. */
  public AHRS gyro;
  public Navx() {
    gyro = new AHRS(SerialPort.Port.kUSB1);
    gyro.zeroYaw();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
