package org.troyargonauts.robot.subystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;

import static org.troyargonauts.robot.Constants.Swerve.*;


public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor, turnMotor;
    private double driveEncoder, turnEncoder;

    private TalonFXConfiguration config = new TalonFXConfiguration();

    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1);
    private Slot0Configs driveConfig = config.Slot0;
    private Slot1Configs turnConfig = config.Slot1;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double chassisAngularOffset;

    public SwerveModule(int driveMotorID, int turnMotorID, double chassisAngularOffset){
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turnMotor.setNeutralMode(NeutralModeValue.Brake);


        driveEncoder = 0;
        turnEncoder = 0;


        
        driveConfig.kP = DRIVE_P;
        driveConfig.kI = DRIVE_I;
        driveConfig.kD = DRIVE_D;

        turnConfig.kP = TURN_P;
        turnConfig.kI = TURN_I;
        turnConfig.kD = TURN_D;

        driveMotor.getConfigurator().apply(config.Slot0);
        turnMotor.getConfigurator().apply(config.Slot1);

        //RESET ENCODERS
        resetEncoders();

        
        this.chassisAngularOffset = chassisAngularOffset;
    }

    @Override
    public void periodic() {
        driveEncoder = driveMotor.getPosition().getValueAsDouble();
        turnEncoder = turnMotor.getPosition().getValueAsDouble();
        //ADD THE CONVERSION FACTOR TO VELOCITY SO THAT IT CAN WORK WITH THE WPILIB API

    }

    /**

     */
    public double convertEncoderToMeters(){
        //DO THIS
        return 0.0;
    }


   /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(getVelocity(),
            new Rotation2d(turnEncoder - chassisAngularOffset));
    }


   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            driveEncoder,
            new Rotation2d(turnEncoder - chassisAngularOffset));
    }


    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(turnEncoder));

        // Command driving and turning motors towards their respective setpoints (velocity and position).
        driveMotor.setControl(velocityVoltage.withVelocity(optimizedDesiredState.speedMetersPerSecond));
        turnMotor.setControl(positionVoltage.withPosition(optimizedDesiredState.angle.getRadians()));

        this.desiredState = desiredState;
    }


    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turnMotor.setPosition(0);
    }

    /*

     */
    public void resetDrive(){
        driveMotor.setPosition(0);
    }

    /**

     */
    public void resetTurn(){
        turnMotor.setPosition(0);
    }

    /**

     */
    public double getVelocity(){
        return driveMotor.getVelocity().getValue();
    }
}
