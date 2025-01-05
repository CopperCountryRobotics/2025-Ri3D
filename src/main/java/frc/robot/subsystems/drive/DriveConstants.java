package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {

  public static final int gyroCANID = 1;

  public static final Rotation3d gyroMountPose = new Rotation3d(0.0, 0.0, 0.0);

  public static final int leftOneCANID = 2;
  public static final int leftTwoCANID = 3;
  public static final int rightOneCANID = 4;
  public static final int rightTwoCANID = 5;

  public static final double driveGearRatio = 12.0;
  public static final Distance driveWheelRadius = Inches.of(4);
  public static final Distance divePositionConversionFactor = Meters
      .of((2.0 * Math.PI * driveWheelRadius.in(Meters)) / driveGearRatio);
  public static final LinearVelocity diveVelocityConversionFactor = MetersPerSecond.of(driveGearRatio / 60.0);

  public static final Distance trackWidth = Inches.of(1.0);

}