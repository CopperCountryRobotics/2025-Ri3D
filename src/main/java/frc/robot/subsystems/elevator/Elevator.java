package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Elevator extends SubsystemBase{
    private final SparkMax heightMotor;

    private final SparkClosedLoopController heightPID;


    public Elevator(){
        heightMotor = new SparkMax(ElevatorConstants.heightMotorCANID, MotorType.kBrushless);
        heightPID = heightMotor.getClosedLoopController();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);
        motorConfig.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
            

        SparkUtil.tryUntilOk(
            5, 
            () -> heightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );        
    }

    public boolean hasHitHardStop() {
        return 
            heightMotor.getOutputCurrent() > ElevatorConstants.autoStopCurrentThreshold
            && 
            heightMotor.getEncoder().getVelocity() < ElevatorConstants.autoStopVelocityThreshold.in(MetersPerSecond);
    }

    /**
     * Sets the elevators setpoint.
     * @param height Position to move towards
     */
    public void setPosition(double height){
        heightPID.setReference(height, ControlType.kPosition);
    }
}
