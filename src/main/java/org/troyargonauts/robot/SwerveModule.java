// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import org.troyargonauts.robot.Constants.*;

public class SwerveModule {
private final TalonFX driveMotor;
private final TalonFX turnMotor;

  private double driveEncoder;
  private double turnEncoder;

  private final PositionVoltage positionvVoltage = new PositionVoltage(0).withSlot(0);
  public final Slot0Configs driveConfig = new Slot0Configs();
  public final Slot1Configs turnConfig = new Slot1Configs();

  private final PIDController drivePIDController;
  private final PIDController turnPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingId, int turningId, double chassisAngularOffset) {

    driveConfig.kP = Swerve.driveP;
    driveConfig.kI = Swerve.driveI;
    driveConfig.kD = Swerve.driveD;

    turnConfig.kP = Swerve.turnP;
    turnConfig.kI = Swerve.turnI;
    turnConfig.kD = Swerve.turnD;

    driveMotor = new TalonFX(drivingId);
    turnMotor = new TalonFX(turningId);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    resetEncoders();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getPosition().getValueAsDouble();
    turnEncoder = turnMotor.getPosition().getValueAsDouble();
    drivePIDController = driveMotor.getPIDController();
    turnPIDController = turnMotor.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turnPIDController.setFeedbackDevice(turnEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePIDController.setP(ModuleConstants.kDrivingP);
    drivePIDController.setI(ModuleConstants.kDrivingI);
    drivePIDController.setD(ModuleConstants.kDrivingD);
    drivePIDController.setFF(ModuleConstants.kDrivingFF);
    drivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turnPIDController.setP(ModuleConstants.kTurningP);
    turnPIDController.setI(ModuleConstants.kTurningI);
    turnPIDController.setD(ModuleConstants.kTurningD);
    turnPIDController.setFF(ModuleConstants.kTurningFF);
    turnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

 driveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turnMotor.setIdleMode(ModuleConstants.kturnMotorIdleMode);
 driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turnMotor.setSmartCurrentLimit(ModuleConstants.kturnMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
 driveMotor.burnFlash();
    turnMotor.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turnEncoder.getPosition() - m_chassisAngularOffset));
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
        driveEncoder.getPosition(),
        new Rotation2d(turnEncoder.getPosition() - m_chassisAngularOffset));
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
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turnEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveMotor.setPosition(0);
    turnMotor.setPosition(0);
  }
}