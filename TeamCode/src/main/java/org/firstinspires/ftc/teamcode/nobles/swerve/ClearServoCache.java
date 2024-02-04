package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ClearServoCache extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveServoStorage.hasCachedPositions = false;
    }
}
