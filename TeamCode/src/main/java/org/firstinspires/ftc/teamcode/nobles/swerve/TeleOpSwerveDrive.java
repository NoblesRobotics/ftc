package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpSwerveDrive extends SwerveDrive {
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.203, 0.203),
            new Translation2d(0.203, -0.203),
            new Translation2d(-0.203, 0.203),
            new Translation2d(-0.203, -0.203)
    );
    private final int[] servoToStateMap = new int[] {3, 1, 0, 2};

    public TeleOpSwerveDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void update(double y, double x, double omega) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(-y, -x, omega)
        );

        double maxSpeed = 0;
        for (SwerveModuleState state : states) {
            if (Math.abs(state.speedMetersPerSecond) > maxSpeed) maxSpeed = Math.abs(state.speedMetersPerSecond);
        }

        for (int i = 0; i < 4; i++) {
            modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
            modules[i].updateServo();
            modules[i].setPower(states[i].speedMetersPerSecond / maxSpeed); // why index not mapped????
        }
    }
}
