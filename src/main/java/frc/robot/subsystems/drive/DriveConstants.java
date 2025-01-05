package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class DriveConstants {

  public static final int leftOneCANID = 1;
  public static final int leftTwoCANID = 2;
  public static final int rightOneCANID = 3;
  public static final int rightTwoCANID = 4;

  public static final Distance trackWidth = Inches.of(1.0);

  public static final Mass robotMass = Pounds.of(100);
  public static final MomentOfInertia moi = KilogramSquareMeters.of(1);

  public static final Distance wheelRadius = Inches.of(4);
  public static final LinearVelocity maxVelocity = MetersPerSecond.of(12);
  public static final Current currentLimit = Amps.of(20);
  public static final double wheelCOF = 0;


}