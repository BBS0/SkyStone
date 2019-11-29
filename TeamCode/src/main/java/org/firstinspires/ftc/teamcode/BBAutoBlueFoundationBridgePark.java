package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-Foundation-Blue-Park-Bridge", group="BB")
public class BBAutoBlueFoundationBridgePark extends LinearOpMode
{
    private BBSRobot robot = new BBSRobot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry, this);

        waitForStart();

        sleep(15000);

        robot.RobotMoveY(new Waypoint(0, 60, 0 ), 0.2);
        robot.Stop();

        robot.RobotMoveX(new Waypoint(97, 0, 0 ), 0.4);
        robot.Stop();
    }
}