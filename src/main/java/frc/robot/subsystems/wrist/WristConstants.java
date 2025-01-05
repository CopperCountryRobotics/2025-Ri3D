package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class WristConstants {
    public static final int motorCANId = 7;
    public static final double currentAutoStopThreshold = 15;
    public static final AngularVelocity autoStopVelocityThreshold = RPM.of(10); //TODO measure

    public static final double wristSpeed = 0.75;
}
