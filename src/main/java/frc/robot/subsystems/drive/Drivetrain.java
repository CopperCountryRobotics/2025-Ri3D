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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

  private final SparkMax left1, left2, right1, right2;
  private final RelativeEncoder leftEncoder, rightEncoder;

  private final Pigeon2 gyro;

  private final DifferentialDrive drive;

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    left1 = new SparkMax(DriveConstants.leftOneCANID, MotorType.kBrushless);
    left2 = new SparkMax(DriveConstants.leftTwoCANID, MotorType.kBrushless);

    right1 = new SparkMax(DriveConstants.rightOneCANID, MotorType.kBrushless);
    right2 = new SparkMax(DriveConstants.rightTwoCANID, MotorType.kBrushless);

    leftEncoder = left1.getEncoder();
    rightEncoder = right1.getEncoder();

    var globalConfig = new SparkMaxConfig();
    var leftLeaderConfig = new SparkMaxConfig();
    var rightLeaderConfig = new SparkMaxConfig();
    var leftFollowerConfig = new SparkMaxConfig();
    var rightFollowerConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit((int) DriveConstants.currentLimit.in(Amps))
        .idleMode(IdleMode.kBrake);

    globalConfig
        .encoder
        .positionConversionFactor(DriveConstants.divePositionConversionFactor.in(Meters))
        .velocityConversionFactor(DriveConstants.diveVelocityConversionFactor.in(MetersPerSecond))
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    leftLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    leftFollowerConfig
        .apply(globalConfig)
        .follow(left1);

    rightLeaderConfig
        .apply(globalConfig)
        .inverted(false);

    rightFollowerConfig
        .apply(globalConfig)
        .follow(right1);

    SparkUtil.tryUntilOk(5,
        () -> left1.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> left2.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> right1.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> right2.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    gyro = new Pigeon2(DriveConstants.gyroCANID);

    var gyroConfig = new Pigeon2Configuration();
    gyroConfig.MountPose.MountPoseYaw = DriveConstants.gyroMountPose.getMeasureZ().in(Degrees);
    gyroConfig.MountPose.MountPosePitch = DriveConstants.gyroMountPose.getMeasureY().in(Degrees);
    gyroConfig.MountPose.MountPoseRoll = DriveConstants.gyroMountPose.getMeasureX().in(Degrees);

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
    odometry.update(
      gyro.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition());
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
    odometry.resetPose(reset);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return 
      kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
          leftEncoder.getVelocity(),
          rightEncoder.getVelocity()
        )
      );
  }

  public void driveRobotRelative(ChassisSpeeds speed) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speed);
    left1.set(wheelSpeeds.leftMetersPerSecond / DriveConstants.maxVelocity.in(MetersPerSecond));
    right1.set(wheelSpeeds.rightMetersPerSecond / DriveConstants.maxVelocity.in(MetersPerSecond));
  }

}
