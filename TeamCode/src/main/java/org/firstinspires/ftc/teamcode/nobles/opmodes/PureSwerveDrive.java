package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveModule;
import org.w3c.dom.Attr;

public class PureSwerveDrive extends SwerveDrive {
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(-0.203, -0.203),
            new Translation2d(0.203, -0.203),
            new Translation2d(0.203, 0.203),
            new Translation2d(-0.203, 0.203)
    );
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(0),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    );

    public PureSwerveDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void followPath(Path path) {
        while (!path.isFinished() && !path.timedOut()) {
            Pose2d pose = odometry.updateWithTime(
                    System.nanoTime() / 1e9,
                    Rotation2d.fromDegrees(imu.getAngle()),
                    modules[0].getState(),
                    modules[1].getState(),
                    modules[2].getState(),
                    modules[3].getState()
            );

            double[] speeds = path.loop(pose.getX(), pose.getY(), pose.getHeading());
            System.out.println(speeds[0] + " " + speeds[1] + " " + speeds[2]);
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(speeds[0], speeds[1], speeds[2])
            );

            boolean hasMajorError = false;
            for (int i = 0; i < 4; i++) {
                modules[i].setAngle(states[i].angle.getDegrees());
                modules[i].updateServo();
                hasMajorError |= modules[i].isServoErrorMajor();
            }

            for (int i = 0; i < 4; i++) {
                if (hasMajorError) modules[i].setPower(0);
                else modules[i].setVelocityMetersPerSecond(states[i].speedMetersPerSecond);
            }
        }

        for (SwerveModule module : modules) module.setPower(0);
    }
}
