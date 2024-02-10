package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveModule;

public class FastSwerveDrive extends SwerveDrive {
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(-0.203, -0.203),
            new Translation2d(0.203, -0.203),
            new Translation2d(0.203, 0.203),
            new Translation2d(-0.203, 0.203)
    );

    public FastSwerveDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    private final double TICKS_PER_REV = 300, INCHES_PER_REV = Math.PI * 2.8, METERS_PER_INCH = 0.0254,
            TICKS_PER_METER = TICKS_PER_REV / INCHES_PER_REV / METERS_PER_INCH,
            METERS_PER_TICK = 1 / TICKS_PER_METER;

    public void driveAndTurn(double forwardInches, double targetAngle) {
        double initialDeltaAngle = deltaAngle(imu.getAngle(), targetAngle);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                        forwardInches * METERS_PER_INCH,
                        0,
                        Math.toRadians(initialDeltaAngle)
                )
        );
        for (int i = 0; i < 4; i++) modules[i].setAngle(states[i].angle.getDegrees());
        positionServos();

        for (int i = 0; i < 4; i++) {
            System.out.println("state" + i + " " + states[i].angle.getDegrees() + " " + states[i].speedMetersPerSecond);
        }

        double maxSpeed = 0;
        for (SwerveModuleState state : states) maxSpeed = Math.max(maxSpeed, state.speedMetersPerSecond);
        double totalDriveInches = forwardInches + Math.toRadians(initialDeltaAngle) * 8.75;

        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 2) {
            for (int i = 0; i < 4; i++) {
                modules[i].setPower(states[i].speedMetersPerSecond / maxSpeed * 0.4);
                modules[i].updateServo();
            }
        }

        for (SwerveModule module : modules) module.setPower(0);
    }

    private int[] readAllPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
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
