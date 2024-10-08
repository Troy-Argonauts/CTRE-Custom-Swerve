// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import org.troyargonauts.common.input.Gamepad;
import org.troyargonauts.common.input.gamepads.AutoGamepad;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static final Gamepad driver = new AutoGamepad(Constants.Controllers.DRIVER);
    public static final Gamepad operator = new AutoGamepad(Constants.Controllers.OPERATOR);

    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
    }

    /**
     * Use this method to define your controller->command mappings.
     */
    private void configureBindings() {
        //Example Joystick Input Command
        //Pulled for Troy-Argonauts/Butters
//            Robot.getDrivetrain().setDefaultCommand(
//                    new RunCommand(
//                            () -> {
//                                double speed = IStream.create(driver::getLeftY)
//                                        .filtered(x -> OMath.deadband(x, Constants.DriveTrain.DEADBAND))
//                                        .get();
//                                double angle = IStream.create(driver::getRightX)
//                                        .filtered(x -> OMath.deadband(x, Constants.DriveTrain.DEADBAND))
//                                        .get();
//                                Robot.getDrivetrain().cheesyDrive(speed, angle, 1);
//                            }, Robot.getDrivetrain()
//                    )
//            );

        //Example Button Input Command
        //Pulled for Troy-Argonauts/Butters
//        driver.getRightBumper().whileTrue(
//                new InstantCommand(() -> Robot.getDrivetrain().getDualSpeedTransmission().disableAutomaticShifting())
//                        .andThen(new InstantCommand(() -> getDriver().setRumble(1.0, 0.5)))
//        );

        Robot.getDrivetrain().setDefaultCommand(
                new RunCommand(
                        () -> {
                            double xSpeed = IStream.create(driver::getLeftX)
                                    .filtered(x -> OMath.deadband(x, Constants.Controllers.DEADBAND))
                                    .get();
                            double ySpeed = IStream.create(driver::getLeftY)
                                    .filtered(y -> OMath.deadband(y, Constants.Controllers.DEADBAND))
                                    .get();
                            double rot = IStream.create(driver::getRightX)
                                    .filtered(x -> OMath.deadband(x, Constants.Controllers.DEADBAND))
                                    .get();
                            Robot.getDrivetrain().drive(xSpeed, ySpeed, rot, true, true);
                        }, Robot.getDrivetrain()
                )
        );
    }

    public static Gamepad getDriver() {
        return driver;
    }

    public static Gamepad getOperator() {
        return operator;
    }
}