package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;

public class SwerveDrive {
    protected final SwerveModule[] modules;
    protected final SimpleIMU imu;

    public SwerveDrive(HardwareMap hardwareMap) {
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
            AttributeTelemetry.set("Calibration Time", String.valueOf(timer.seconds()));
        }
        SwerveServoStorage.hasCachedPositions = true;
        AttributeTelemetry.remove("Calibration Time");
    }

    public void resetServos() {
        for (SwerveModule module : modules) {
            module.setPower(0);
            module.resetAngle();
        }
        positionServos();
    }

    protected void positionServos() {
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
