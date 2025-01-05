package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.AllianceUtil;

public class AutoGenerator {

    private final SendableChooser<Command> commandSelector;
    
    public AutoGenerator(Drivetrain drive) {

        ModuleConfig moduleConfig = new ModuleConfig(
            DriveConstants.driveWheelRadius, 
            DriveConstants.maxVelocity,
            DriveConstants.wheelCOF,
            DCMotor.getNEO(2).withReduction(DriveConstants.driveGearRatio),
            DriveConstants.currentLimit,
            4
        );

        RobotConfig robotConfig = new RobotConfig(
            DriveConstants.robotMass,
            DriveConstants.moi,
            moduleConfig,
            DriveConstants.trackWidth
        );

        AutoBuilder.configure(
            drive::getCurrentPose,
            drive::resetPose,
            drive::getRobotRelativeSpeeds,
            (speed, ff) -> {drive.driveRobotRelative(speed);},
            new PPLTVController(Constants.deltaTime),
            robotConfig,
            AllianceUtil::isRedAlliance,
            drive
        );

        registerCommands();
        commandSelector = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Auto").add(commandSelector);
    }

    private void registerCommands() {
        //TODO subsystem integration
    }

    public Command getSelectedCommand() {
        return commandSelector.getSelected();
    }



}
