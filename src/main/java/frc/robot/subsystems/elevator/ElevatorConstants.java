package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int heightMotorCANID = 6;
    public static final int heightMotor2CANID = 7;
    public static final double positionConversionFactor = 1; //TODO: measure this

    public static final double autoStopCurrentThreshold = 15;
    public static final double autoStopVelocityThreshold = 0.1; //TODO measure

    public static final double topHeight = 0;
    public static final double middleHeight = -45.7;
    public static final double bottomHeight = 0;
}
