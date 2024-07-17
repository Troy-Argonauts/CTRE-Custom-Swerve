package org.troyargonauts.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelDrive extends SubsystemBase {
    
    private TalonFX angleMotor = new TalonFX(0)

    public WheelDrive(){

    }
}
