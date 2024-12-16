// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {


  private PWMTalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2;

  private DifferentialDrive drive;
  private MotorControllerGroup leftGroup;
  private MotorControllerGroup rightGroup;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftDrive1 = new PWMTalonSRX(0);
    leftDrive2 = new PWMTalonSRX(1);
    rightDrive1 = new PWMTalonSRX(2);
    rightDrive2 = new PWMTalonSRX(3);

    leftDrive1.setInverted(true);
    leftDrive2.setInverted(true);

    leftGroup = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightGroup = new MotorControllerGroup(rightDrive1, rightDrive2);

    drive = new DifferentialDrive(leftGroup, rightGroup);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void tankDrive (double left, double right){
    drive.tankDrive(left, right, true);
  }

  public void arcadeDrive(double fwd, double turn){
    drive.arcadeDrive(fwd, turn, true);
  }
}
