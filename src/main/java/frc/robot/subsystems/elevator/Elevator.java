package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.SparkUtil;

public class Elevator extends SubsystemBase{
    private final SparkMax heightMotor;
    private final SparkMax armMotor;
    private final SparkClosedLoopController closedLoopController;
    private final SparkClosedLoopController armLoopController;
    public Elevator(){
        heightMotor = new SparkMax(ElevatorConstants.heightMotorCANID, MotorType.kBrushless);
        armMotor = new SparkMax(ElevatorConstants.armMotorCANID, MotorType.kBrushless);

        closedLoopController = heightMotor.getClosedLoopController();
        armLoopController = armMotor.getClosedLoopController();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);
        motorConfig.encoder.positionConversionFactor(ElevatorConstants.positionConversionFactor);
        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for height control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
            

        SparkUtil.tryUntilOk(5, 
        () -> heightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        motorConfig.encoder.positionConversionFactor(ElevatorConstants.armPositionConversionFactor);
        motorConfig.closedLoop
            // Set PID values for arm control in slot 1
            .p(0.1, ClosedLoopSlot.kSlot0)
            .i(0, ClosedLoopSlot.kSlot0)
            .d(0, ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        SparkUtil.tryUntilOk(5, 
        () -> armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        
        
    }

    /**
     * Set the motion of the elevator. Positive is up.
     * @param speed
     */
    public void run(double speed){
        heightMotor.set(speed);
    }

    /**
     * Sets the elevator and accompanying arm's setpoints. I have never used Rev's built-in PID, so I can only hope this works.
     * @param heightPosition Elevator's setpoint
     * @param armAngle Arm's setpoint
     */
    public void setPose(double heightPosition, double armAngle){
        closedLoopController.setReference(heightPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        armLoopController.setReference(armAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
