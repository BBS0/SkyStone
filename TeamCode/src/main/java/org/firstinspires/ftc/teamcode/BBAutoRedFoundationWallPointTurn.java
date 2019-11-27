package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-Foundation-Red-Point-Wall", group="BB")
public class BBAutoRedFoundationWallPointTurn extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBHooks hooks = new BBHooks();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry, this);
        hooks.init(hardwareMap);

        hooks.UnLatched();

        robot.RobotMoveY(new Waypoint(0, 20, 0 ), 0.3);
        robot.Stop();

        robot.RobotMoveX(new Waypoint(45, 0, 0 ), 0.3);
        robot.Stop();

        robot.RobotMoveY(new Waypoint(0, 52, 0 ), 0.3);
        robot.Stop();

        hooks.Latched();
        sleep( 1000);

        robot.RobotMoveY(new Waypoint(0, -20, 0 ), 0.5);
        robot.Stop();

        robot.turnRight(10, 0.5);
        robot.Stop();

        robot.RobotMoveY(new Waypoint(0, -20, 0 ), 0.5);
        robot.Stop();

        robot.turnRight(10, 0.5);
        robot.Stop();

        robot.RobotMoveY(new Waypoint(0, -20, 0 ), 0.5);
        robot.Stop();

        robot.turnRight(10, 0.5);
        robot.Stop();

        robot.RobotMoveY(new Waypoint(0, -20, 0 ), 0.5);
        robot.Stop();

        robot.turnRight(10, 0.5);
        robot.Stop();

        robot.RobotMoveY(new Waypoint(0, -20, 0 ), 0.5);
        robot.Stop();

        robot.turnRight(50, 0.5);
        robot.Stop();

    }


}
