package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
    public static final int heightMotorCANID = 6;
    public static final double positionConversionFactor = 1; //TODO: measure this

    public static final double autoStopCurrentThreshold = 15;
    public static final LinearVelocity autoStopVelocityThreshold = MetersPerSecond.of(0.1); //TODO measure

    public static final double topHeight = 0;
    public static final double middleHeight = 0;
    public static final double bottomHeight = 0;
}
