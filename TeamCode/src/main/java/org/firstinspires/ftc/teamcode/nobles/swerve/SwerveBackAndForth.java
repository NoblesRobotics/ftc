package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SwerveBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        drive.calibrateServos();

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(48);
            sleep(3000);
            drive.drive(-48);
            sleep(3000);
        }
    }
}
