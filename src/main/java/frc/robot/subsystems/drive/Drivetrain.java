// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.SparkUtil;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax left1, left2, right1, right2;
 private final RelativeEncoder leftEncoder, rightEncoder;

  private final Pigeon2 gyro;

  private final DifferentialDrive drive;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    left1 = new CANSparkMax(DriveConstants.leftOneCANID, MotorType.kBrushless);
    left2 = new CANSparkMax(DriveConstants.leftTwoCANID, MotorType.kBrushless);

    right1 = new CANSparkMax(DriveConstants.rightOneCANID, MotorType.kBrushless);
    right2 = new CANSparkMax(DriveConstants.rightTwoCANID, MotorType.kBrushless);

    leftEncoder = left1.getEncoder();
    rightEncoder = right1.getEncoder();

    
    left1.setSmartCurrentLimit(((int) DriveConstants.currentLimit));
    left1.setIdleMode(IdleMode.kBrake);
    leftEncoder.setPositionConversionFactor(DriveConstants.divePositionConversionFactor);
    leftEncoder.setVelocityConversionFactor(DriveConstants.diveVelocityConversionFactor);

    left2.setSmartCurrentLimit(((int) DriveConstants.currentLimit));
    left2.setIdleMode(IdleMode.kBrake);
    left2.follow(left1);



    right1.setSmartCurrentLimit(((int) DriveConstants.currentLimit));
    right1.setIdleMode(IdleMode.kBrake);
    rightEncoder.setPositionConversionFactor(DriveConstants.divePositionConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.diveVelocityConversionFactor);

    right2.setSmartCurrentLimit(((int) DriveConstants.currentLimit));
    right2.setIdleMode(IdleMode.kBrake);
    right2.follow(right1);


    gyro = new Pigeon2(DriveConstants.gyroCANID);

    var gyroConfig = new Pigeon2Configuration();
    gyroConfig.MountPose.MountPoseYaw = DriveConstants.gyroMountPose.getZ();
    gyroConfig.MountPose.MountPosePitch = DriveConstants.gyroMountPose.getY();
    gyroConfig.MountPose.MountPoseRoll = DriveConstants.gyroMountPose.getX();

    PhoenixUtil.tryUntilOk(5, () -> gyro.getConfigurator().apply(gyroConfig));

    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    odometry = new DifferentialDriveOdometry(
        new Rotation2d(),
        0.0,
        0.0);

    drive = new DifferentialDrive(left1, right1);
  }

  @Override
  public void periodic() {
   // odometry.update(
     // gyro.getRotation2d(), 
      //leftEncoder.getPosition(), 
     // rightEncoder.getPosition());
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right, true);
  }

  public void arcadeDrive(double fwd, double turn) {
    drive.arcadeDrive(fwd, turn, true);
  }

  public Pose2d getCurrentPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d reset) {
    //odometry.reset;
  }

  // public ChassisSpeeds getRobotRelativeSpeeds() {
  //   return 
  //     kinematics.toChassisSpeeds(
  //       new DifferentialDriveWheelSpeeds(
  //         leftEncoder.getVelocity(),
  //         rightEncoder.getVelocity()
  //       )
  //     );
  // }

  // public void driveRobotRelative(ChassisSpeeds speed) {
  //   var wheelSpeeds = kinematics.toWheelSpeeds(speed);
  //   left1.set(wheelSpeeds.leftMetersPerSecond / DriveConstants.maxVelocity.in(MetersPerSecond));
  //   right1.set(wheelSpeeds.rightMetersPerSecond / DriveConstants.maxVelocity.in(MetersPerSecond));
  // }


}
