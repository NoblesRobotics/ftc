package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.TeleOp;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

@Config
public class AutoSwerveDrive {
    private final double TICKS_PER_REV = 300, INCHES_PER_REV = Math.PI * 2.8, TOLERANCE = 0.05, ANGLE_TOLERANCE = 1.5;

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

    public static PIDImpl drivePid = new PIDImpl(10, 0, 0);

    public void drive(double inches) {
        drivePid.reset();

        int targetDeltaTick = (int) (inches / INCHES_PER_REV * TICKS_PER_REV);
        int[] startPositions = readAllPositions();
        int loopsNoneBusy = 0;
        while (true) {
            int[] currentPositions = readAllPositions();
            double headingError = deltaAngle(imu.getAngle(), forwardAngle);
            double imuAdjustMagnitude = headingError * 0.05;
            double[] imuAdjusts = new double[] {
                imuAdjustMagnitude,
                imuAdjustMagnitude,
                -imuAdjustMagnitude,
                -imuAdjustMagnitude
            };
            boolean anyBusy = false;
            for (int i = 0; i < 4; i++) {
                double deltaTick = Math.abs(currentPositions[i] - startPositions[i]);
                double driveError = Math.abs(targetDeltaTick - deltaTick) / targetDeltaTick;
                double drivePower;
                if (driveError <= TOLERANCE) {
                    drivePower = 0;
                } else {
                    drivePower = Math.signum(targetDeltaTick - deltaTick) * Math.max(drivePid.update(driveError), 0.2);
                    anyBusy = true;
                }
                modules[i].setPower(drivePower + imuAdjusts[i]);
                modules[i].updateServo();
            }
            if (!anyBusy) {
                loopsNoneBusy++;
                if (loopsNoneBusy > 10) break;
            } else {
                loopsNoneBusy = 0;
            }
        }
    }

    //public static PIDImpl headingPid = new PIDImpl(0.075, 0, 0);

    public void turn(double targetAngle) {
        double[] swerveAngles = {-45, 45, 135, -135};
        for (int i = 0; i < 4; i++) modules[i].setAngle(swerveAngles[i]);
        positionServos();

        while (true) {
            double headingError = deltaAngle(imu.getAngle(), targetAngle);
            double imuAdjustMagnitude = piecewiseSlowVelCurve(Math.abs(headingError) / 90);
            boolean anyBusy = false;
            for (int i = 0; i < 4; i++) {
                double drivePower;
                if (Math.abs(headingError) <= ANGLE_TOLERANCE) {
                    drivePower = 0;
                } else {
                    drivePower = Math.signum(headingError) * imuAdjustMagnitude;
                    anyBusy = true;
                }
                TelemetryStatic.telemetry.addData("drivePower", drivePower);
                TelemetryStatic.telemetry.update();
                modules[i].setPower(drivePower);
                modules[i].updateServo();
            }
            if (!anyBusy) break;
        }

        setAngle(0);
    }

    private int[] readAllPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    private double piecewiseSlowVelCurve(double error) {
        if (error > 0.35) return 0.5;
        else return Math.max(error / 0.7, 0.25);
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
