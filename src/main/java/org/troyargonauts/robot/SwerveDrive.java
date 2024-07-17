package org.troyargonauts.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase{
    private double length = 6;
    private double width = 1;
    //MAKE SURE TO ADD THIS TO CONSTANT


    public SwerveDrive(){

    }

    public void drive (double x1, double y1, double x2) {
        double r = Math.sqrt ((length * length) + (width * width));
        y1 *= -1;
        
        double a = x1 - x2 * (length / r);
        double b = x1 + x2 * (length / r);
        double c = y1 - x2 * (width / r);
        double d = y1 + x2 * (width / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
    }
}
