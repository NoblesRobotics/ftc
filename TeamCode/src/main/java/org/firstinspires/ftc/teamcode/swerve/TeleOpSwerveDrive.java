package org.firstinspires.ftc.teamcode.swerve;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpSwerveDrive extends SwerveDrive {
    // Each Translation2d represents one of the wheels
    // 0.203 meters ~= 8 inches = half the length of the robot = distance of each wheel to center line
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(-0.203, -0.203),
            new Translation2d(0.203, -0.203),
            new Translation2d(0.203, 0.203),
            new Translation2d(-0.203, 0.203)
    );

    public TeleOpSwerveDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void update(double y, double x, double omega) {
        // states has length 4, one element per wheel
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(-y, -x, omega)
        );

        double maxSpeed = 0;
        for (SwerveModuleState state : states) {
            if (Math.abs(state.speedMetersPerSecond) > maxSpeed) maxSpeed = Math.abs(state.speedMetersPerSecond);
        }

        for (int i = 0; i < 4; i++) {
            modules[i].setAngle(states[i].angle.getDegrees());
            modules[i].updateServo();

            // Power to a motor must be between -1.0 and 1.0, a value greater than 1.0 will be
            // treated as 1.0
            // It's important that the balance of motor powers is maintained, so scale every motor's
            // power down so the fastest motor goes with power 1.0
            modules[i].setPower(states[i].speedMetersPerSecond / maxSpeed);
        }
    }
}
