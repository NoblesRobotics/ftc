package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveModule {
    private final SwerveServo servo;
    private final DcMotorEx motor;

    private int powerSign = 1;

    public SwerveModule(HardwareMap hardwareMap, int index, DcMotorSimple.Direction motorDirection) {
        servo = new SwerveServo(hardwareMap, index);
        motor = hardwareMap.get(DcMotorEx.class, "motor" + index);
        motor.setDirection(motorDirection);
    }

    public void calibrateServo() {
        servo.calibrate();
    }

    public void setAngle(double degrees) {
        double backwardsDeg = normalizeAngle(degrees + 180.);
        double currentDeg = normalizeAngle(servo.getAngle());
        if (Math.abs(deltaAngle(backwardsDeg, currentDeg)) < Math.abs(deltaAngle(degrees, currentDeg))) {
            degrees = backwardsDeg;
            powerSign = -1;
        } else {
            powerSign = 1;
        }
        servo.setAngle(getClosestEquivalent(degrees));
    }

    public void resetAngle() {
        powerSign = 1;
        servo.setAngle(getClosestEquivalent(0));
    }

    public double getAngle() {
        return normalizeAngle(servo.getAngle());
    }

    public void updateServo() {
        servo.update();
    }

    public boolean isServoMoving() {
        return servo.isMoving();
    }

    public void setPower(double power) {
        motor.setPower(power * powerSign);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    private double getClosestEquivalent(double degrees) {
        double path = deltaAngle(degrees, normalizeAngle(servo.getAngle()));
        return servo.getAngle() + path;
    }

    private double normalizeAngle(double degrees) {
        while (degrees < -180 || degrees > 180) {
            degrees -= Math.signum(degrees) * 360;
        }
        return degrees;
    }

    private double deltaAngle(double aDeg, double bDeg) {
        if (aDeg > bDeg) return -deltaAngle(bDeg, aDeg);

        aDeg += 180.;
        bDeg += 180.;
        double normal = -(bDeg - aDeg);
        double around = (360 - bDeg) + aDeg;

        if (Math.abs(normal) <= Math.abs(around)) return normal;
        else return around;
    }
}