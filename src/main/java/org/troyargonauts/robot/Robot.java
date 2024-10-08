// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.subystems.SwerveSubsystem;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
    private Command autonomousCommand;
    private static SwerveSubsystem drivetrain;


    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        DataLogManager.start("/media/sda1/logs");

        new RobotContainer();
        drivetrain = new SwerveSubsystem();

        CameraServer.startAutomaticCapture().setFPS(14);

        SmartDashboard.putData("Autonomous modes", chooser);
        chooser.addOption("Nothing", new WaitCommand(15));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    public static SwerveSubsystem getDrivetrain(){
        if (drivetrain == null){
            drivetrain = new SwerveSubsystem();
        }
        return drivetrain;
    }

}
