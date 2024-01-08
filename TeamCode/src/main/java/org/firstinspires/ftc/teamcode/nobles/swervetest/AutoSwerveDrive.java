package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoSwerveDrive {
    private final double TICKS_PER_REV = 315, INCHES_PER_REV = Math.PI * 2.8, TOLERANCE = 0.02;

    private final SwerveModule[] modules;
    private final SimpleIMU imu;

    private double forwardAngle = 0.;

    public AutoSwerveDrive(HardwareMap hardwareMap) {
        modules = new SwerveModule[] {
                new SwerveModule(hardwareMap, 0, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 1, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 2, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 3, DcMotorSimple.Direction.REVERSE)
        };
        imu = new SimpleIMU(hardwareMap);
    }

    public void calibrateServos() {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 3) {
            for (SwerveModule module : modules) module.calibrateServo();
        }
    }

    public void drive(double inches) {
        int targetDeltaTick = (int) (inches / INCHES_PER_REV * TICKS_PER_REV);
        int[] startPositions = readAllPositions();
        while (true) {
            int[] currentPositions = readAllPositions();
            double imuAdjustMagnitude = deltaAngle(imu.getAngle(), forwardAngle) * 0.01;
            double[] imuAdjusts = new double[] {
                imuAdjustMagnitude,
                imuAdjustMagnitude,
                -imuAdjustMagnitude,
                -imuAdjustMagnitude
            };
            boolean anyBusy = false;
            for (int i = 0; i < 4; i++) {
                double deltaTick = Math.abs(currentPositions[i] - startPositions[i]);
                double error = Math.abs(targetDeltaTick - deltaTick) / targetDeltaTick;
                double drivePower;
                if (error <= TOLERANCE) {
                    drivePower = 0;
                } else {
                    drivePower = Math.signum(targetDeltaTick - deltaTick) * piecewiseSlowVelCurve(error);
                    anyBusy = true;
                }
                modules[i].setPower(drivePower + imuAdjusts[i]);
                modules[i].updateServo();
            }
            if (!anyBusy) break;
        }
    }

    private int[] readAllPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    private double piecewiseSlowVelCurve(double error) {
        if (error > 0.35) return 1;
        else return Math.max(error / 0.35, 0.5);
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

    public void setAngle(double degrees) {
        for (SwerveModule module : modules) module.setAngle(degrees);
        positionServos();
    }

    private void positionServos() {
        while (true) {
            boolean anyBusy = false;
            for (SwerveModule module : modules) {
                module.updateServo();
                if (module.isServoMoving()) anyBusy = true;
            }
            if (!anyBusy) break;
        }
    }
}
