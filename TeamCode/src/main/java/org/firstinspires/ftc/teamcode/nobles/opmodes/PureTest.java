package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;

@TeleOp
public class PureTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        PureSwerveDrive drive = new PureSwerveDrive(hardwareMap);
        drive.calibrateServos();

        Waypoint p1 = new StartWaypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Waypoint p2 = new PointTurnWaypoint(0.75, 0, Math.toRadians(90), 1, 1, 0.1, 0.1);
        Waypoint p3 = new EndWaypoint(new Pose2d(1.5, 0, Rotation2d.fromDegrees(90)), 1, 1, 0.1, 0.1, 0.1);
        Path path = new Path(p1, p2, p3);
        path.init();

        waitForStart();

        drive.followPath(path);
    }
}
