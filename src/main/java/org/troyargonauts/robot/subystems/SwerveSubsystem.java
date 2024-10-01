package org.troyargonauts.robot.subystems;

import static org.troyargonauts.robot.Constants.Swerve.BACK_LEFT_CHASSIS_ANGULAR_OFFSET;
import static org.troyargonauts.robot.Constants.Swerve.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_LEFT_DRIVING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_LEFT_TURNING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_RIGHT_DRIVING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.FRONT_RIGHT_TURNING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.BACK_LEFT_DRIVING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.BACK_LEFT_TURNING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.BACK_RIGHT_DRIVING_CAN_ID;
import static org.troyargonauts.robot.Constants.Swerve.BACK_RIGHT_TURNING_CAN_ID;

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
        FRONT_LEFT_DRIVING_CAN_ID,
        FRONT_LEFT_TURNING_CAN_ID,
        FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final SwerveModule frontRightModule = new SwerveModule(
        FRONT_RIGHT_DRIVING_CAN_ID,
        FRONT_RIGHT_TURNING_CAN_ID,
        FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final SwerveModule backLeftModule = new SwerveModule(
        BACK_LEFT_DRIVING_CAN_ID,
        BACK_LEFT_TURNING_CAN_ID,
        BACK_LEFT_CHASSIS_ANGULAR_OFFSET);
        
    private final SwerveModule backRightModule = new SwerveModule(
        BACK_RIGHT_DRIVING_CAN_ID,
        BACK_RIGHT_TURNING_CAN_ID,
        BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);


    // The gyro sensor
    public final Pigeon2 gyro = new Pigeon2(11);


    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(Swerve.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(Swerve.ROTATIONAL_SLEW_RATE);
    private double prevTime = WPIUtilJNI.now() * 1e-6;


    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Swerve.DRIVE_KINEMATICS,
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


    /**
     * Updates the odometry to the current positions of all the wheels and gyro, outputs the turn and drive encoder values for 
     * each swerve module, and also outputs the gyro angle.
     */
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
        
        SmartDashboard.putNumber("FL Drive Encoder", frontLeftModule.driveEncoder);
        SmartDashboard.putNumber("FR Drive Encoder", frontRightModule.driveEncoder);
        SmartDashboard.putNumber("BL Drive Encoder", backLeftModule.driveEncoder);
        SmartDashboard.putNumber("BR Drive Encoder", backRightModule.driveEncoder);

        SmartDashboard.putNumber("FL Turn Encoder", frontLeftModule.turnEncoder);
        SmartDashboard.putNumber("FR Turn Encoder", frontRightModule.turnEncoder);
        SmartDashboard.putNumber("BL Turn Encoder", backLeftModule.turnEncoder);
        SmartDashboard.putNumber("BR Turn Encoder", backRightModule.turnEncoder);
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
                directionSlewRate = Math.abs(Swerve.DIRECTION_SLEW_RATE / currentTranslationMag);
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


        } 
        else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = currentRotation * Swerve.MAX_ANGULAR_SPEED;

        var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
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
            desiredStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }


    /** Resets the drive and turn encoders to a position of 0. */
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }

    /** Resets the turn encoders to a position of 0. */
    public void resetTurnEncoders() {
        frontLeftModule.resetTurnEncoder();
        frontRightModule.resetTurnEncoder();
        backLeftModule.resetTurnEncoder();
        backRightModule.resetTurnEncoder();
    }

    /** Resets the drive encoders to a position of 0. */
    public void resetDriveEncoders() {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();
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
        return gyro.getRate() * (Swerve.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the forward position of the robot in meters.
     * @return Forward position of the robot in meters
     */
    public double getForwardPosition() {
        return (-(getFLDrivePosition() + getFRDrivePosition() + getBLDrivePosition() + getBRDrivePosition()) / 4);
    }

    /**
     * Returns the strafe position of the robot in meters
     * @return Stafe position of the robot in meters
     */
    public double getStrafePosition() {
        return ((getFLDrivePosition() - getFRDrivePosition() - getBLDrivePosition() + getBRDrivePosition()) / 4);
    }

    /**
     * Returns the front left swerve module's drive encoder position in meters.
     * @return Front left swerve module's drive encoder position in meters
     */
    public double getFLDrivePosition(){
        return frontLeftModule.getPosition().distanceMeters;
    }

    /**
     * Returns the front right swerve module's drive encoder position in meters.
     * @return Front right swerve module's drive encoder position in meters
     */
    public double getFRDrivePosition(){
        return frontRightModule.getPosition().distanceMeters;
    }

    /**
     * Returns the back left swerve module's drive encoder position in meters.
     * @return Back left swerve module's drive encoder position in meters
     */
    public double getBLDrivePosition(){
        return backLeftModule.getPosition().distanceMeters;
    }


    /**
     * Returns the back right swerve module's drive encoder position in meters.
     * @return Back right swerve module's drive encoder position in meters
     */
    public double getBRDrivePosition(){
        return backRightModule.getPosition().distanceMeters;
    }

    /**
     * Returns the front right swerve module's turn encoder position in meters.
     * @return Front right swerve module's turn encoder position in meters
     */
    public Rotation2d getFRTurnPosition(){
        return frontRightModule.angle;
    }

    /**
     * Returns front left the swerve module's turn encoder position in meters.
     * @return Front left swerve module's turn encoder position in meters
     */
    public Rotation2d getFLTurnPosition(){
        return frontLeftModule.angle;
    }

    /**
     * Returns the back right swerve module's turn encoder position in meters.
     * @return Back right swerve module's turn encoder position in meters
     */
    public Rotation2d getBRTurnPosition(){
        return backRightModule.angle;
    }

    /**
     * Returns the back left swerve module's turn encoder position in meters.
     * @return Back left swerve module's turn encoder position in meters
     */
    public Rotation2d getBLTurnPosition(){
        return backLeftModule.angle;
    }
}