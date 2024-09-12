package org.troyargonauts.robot.subystems;

import org.troyargonauts.common.swerve.SwerveUtils;
import org.troyargonauts.robot.Constants.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    // Create SwerveModules
    private final SwerveModule frontLeftModule = new SwerveModule(
        0,
        0,
        0);

    private final SwerveModule frontRightModule = new SwerveModule(
        0,
        0,
        0);

    private final SwerveModule backLeftModule = new SwerveModule(
        0,
        0,
        0);
        
    private final SwerveModule backRightModule = new SwerveModule(
        0,
        0,
        0);


    // The gyro sensor
    public final Pigeon2 gyro = new Pigeon2(11);


    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(Swerve.kMagnitudeSlewRate);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(Swerve.kRotationalSlewRate);
    private double prevTime = WPIUtilJNI.now() * 1e-6;


    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Swerve.kDriveKinematics,
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
    });


    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem() {
    }   


    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            });

        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }



    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            pose);
    }


   /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(Swerve.kDirectionSlewRate / currentTranslationMag);
        } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }
        

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
        if (angleDif < 0.45*Math.PI) {
            currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
        else if (angleDif > 0.85*Math.PI) {
            if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
            // keep currentTranslationDir unchanged
            currentTranslationMag = magLimiter.calculate(0.0);
            }
            else {
            currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            }
        }
        else {
            currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            currentTranslationMag = magLimiter.calculate(0.0);
        }
        prevTime = currentTime;
        
        xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
        ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
        currentRotation = rotLimiter.calculate(rot);


        } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * Swerve.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * Swerve.kMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * Swerve.kMaxAngularSpeed;

        var swerveModuleStates = Swerve.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Swerve.kMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }


    /**
   * Sets the wheels into an X formation to prevent movement.
   */
    public void setX() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }


   /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, Swerve.kMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }


    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.setYaw(0);
    }

   /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
    public double getTurnRate() {
        return gyro.getRate() * (Swerve.kGyroReversed ? -1.0 : 1.0);
    }

    public double getForwardEncoder() {
        return (-(frontLeftModule.getPosition().distanceMeters + frontRightModule.getPosition().distanceMeters + backLeftModule.getPosition().distanceMeters + backRightModule.getPosition().distanceMeters) / 4);
    }

    public double getStrafeEncoder() {
        return ((frontLeftModule.getPosition().distanceMeters - frontRightModule.getPosition().distanceMeters - backLeftModule.getPosition().distanceMeters + backRightModule.getPosition().distanceMeters) / 4);
    }

    public double getFLEncoder(){
        return frontLeftModule.getPosition().distanceMeters;
    }

    public double getFREncoder(){
        return frontRightModule.getPosition().distanceMeters;
    }

    public double getBLEncoder(){
        return backLeftModule.getPosition().distanceMeters;
    }

    public double getBREncoder(){
        return backRightModule.getPosition().distanceMeters;
    }
}
