// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    right2 = new SparkMax(DriveConstants.leftTwoCANID, MotorType.kBrushless);

    leftEncoder = left1.getEncoder();
    rightEncoder = right1.getEncoder();

    var globalConfig = new SparkMaxConfig();
    var rightLeaderConfig = new SparkMaxConfig();
    var leftFollowerConfig = new SparkMaxConfig();
    var rightFollowerConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    globalConfig
        .encoder
        .positionConversionFactor(DriveConstants.divePositionConversionFactor.in(Meters))
        .velocityConversionFactor(DriveConstants.diveVelocityConversionFactor.in(MetersPerSecond))
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    leftFollowerConfig
        .apply(globalConfig)
        .follow(left1);

    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    rightFollowerConfig
        .apply(globalConfig)
        .follow(right1);

    SparkUtil.tryUntilOk(5,
        () -> left1.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> left2.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> right1.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(5,
        () -> right2.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    gyro = new Pigeon2(DriveConstants.gyroCANID);

    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);

    odometry = new DifferentialDriveOdometry(
        new Rotation2d(),
        0.0,
        0.0);

    drive = new DifferentialDrive(left1, right1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right, true);
  }

  public void arcadeDrive(double fwd, double turn) {
    drive.arcadeDrive(fwd, turn, true);
  }
}
