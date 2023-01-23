package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;

public class SwerveDrive {
    private final double L = 20;
    private final double W = 20;
    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;
    private ADXRS450_Gyro gyro;
    private double currentHeading;
    private Joystick controller;

    public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        controller = new Joystick(0);
    }

    public void drive() {
        currentHeading = gyro.getAngle();
        double x1 = controller.getX();
        double y1 = controller.getY();
        double x2 = controller.getTwist();
    
        // Rotate joystick inputs to field frame of reference
        double x1Field = x1 * Math.cos(currentHeading) - y1 * Math.sin(currentHeading);
        double y1Field = x1 * Math.sin(currentHeading) + y1 * Math.cos(currentHeading);
    
        // Convert to polar coordinates
        double speed = Math.sqrt(x1Field * x1Field + y1Field * y1Field);
        double angle = Math.atan2(y1Field, x1Field);
    
        // Scale inputs to match desired maximum speed
        double maxSpeed = 1;
        speed = speed * maxSpeed;
    
        double r = Math.sqrt((L * L) + (W * W));

        double a = x1Field - x2 * (L / r);
        double b = x1Field + x2 * (L / r);
        double c = y1Field - x2 * (W / r);
        double d = y1Field + x2 * (W / r);
    
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));
    
        double backRightAngle = Math.atan2(a, d) / Math.PI;
        double backLeftAngle = Math.atan2(a, c) / Math.PI;
        double frontRightAngle = Math.atan2(b, d) / Math.PI;
        double frontLeftAngle = Math.atan2(b, c) / Math.PI;
    
        backRight.drive(backRightSpeed, backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);
    }
}