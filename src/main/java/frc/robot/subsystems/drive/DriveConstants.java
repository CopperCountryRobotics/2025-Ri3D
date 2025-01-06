package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;

public class DriveConstants {

  public static final int gyroCANID = 5;

  public static final Rotation3d gyroMountPose = new Rotation3d(0.0, 0.0, 0.0);

  public static final int leftOneCANID = 1;
  public static final int leftTwoCANID = 2;
  public static final int rightOneCANID = 3;
  public static final int rightTwoCANID = 4;

  public static final double driveGearRatio = 12.0;
  public static final double driveWheelRadius = Units.inchesToMeters(4.0);
  public static final double divePositionConversionFactor = (2.0 * Math.PI * driveWheelRadius) / driveGearRatio;
  public static final double diveVelocityConversionFactor = driveGearRatio / 60.0;

  public static final double trackWidth = Units.inchesToMeters(1);

  public static final double robotMass = 100; 
  public static final double moi = 1;

  public static final double maxVelocity = 12; //12 meters per second
  public static final double currentLimit = 20; //20 amps
  public static final double wheelCOF = 0;


}