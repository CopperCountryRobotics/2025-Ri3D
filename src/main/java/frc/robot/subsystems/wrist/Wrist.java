package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Wrist extends SubsystemBase {
    
    private final SparkMax motor;

    public Wrist() {
        motor = new SparkMax(WristConstants.motorCANId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(0).idleMode(IdleMode.kBrake);
        config.encoder.velocityConversionFactor(0);

        SparkUtil.tryUntilOk(
            5, 
            () -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    }

    public boolean hasHitHardStop() {
        return 
            motor.getOutputCurrent() > WristConstants.currentAutoStopThreshold 
            &&
            motor.getEncoder().getVelocity() < WristConstants.autoStopVelocityThreshold.in(RPM);
    }

    /**
     * Rotate the wrist. Positive is away from the robot. 
     * @param speed
     */
    public void rotate(double speed) {
        motor.set(speed);
    }
}
