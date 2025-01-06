package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private final CANSparkMax motor;
    private RelativeEncoder encoder;

    public Wrist() {
        motor = new CANSparkMax(WristConstants.motorCANId, MotorType.kBrushless);

        //SparkMaxConfig config = new SparkMaxConfig();
        motor.setSmartCurrentLimit(0);
        motor.setIdleMode(IdleMode.kBrake);
        encoder.setVelocityConversionFactor(0);
    }

    public boolean hasHitHardStop() {
        return 
            motor.getOutputCurrent() > WristConstants.currentAutoStopThreshold 
            &&
            motor.getEncoder().getVelocity() < WristConstants.autoStopVelocityThreshold;
    }

    /**
     * Rotate the wrist. Positive is away from the robot. 
     * @param speed
     */
    public void rotate(double speed) {
        motor.set(speed);
    }
}
