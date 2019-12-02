package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

import java.util.List;

@Autonomous(name="Auto-Foundation-Red-Far", group="BB")
public class BBOpModeAutonomousFoundationRedFar extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBHooks hooks = new BBHooks();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);
        hooks.init(hardwareMap);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //hooks.UnLatched();
        //sleep(1000);
        //work out how far forward we need to move
        //robot.moveForward(150, 0.55);
        //sleep(1000);

        //put down the hooks into the foundation
        //hooks.Latched();

        hooks.UnLatched();

        robot.RobotMoveX(new Waypoint(20, 0, 0 ), 0.3);
        robot.Stop();
        sleep(100);

        robot.RobotMoveY(new Waypoint(0, 70, 0 ), 0.3);
        robot.Stop();
        sleep(100);

        hooks.Latched();
        sleep(1000);

        robot.RobotMoveY(new Waypoint(0, -65, 0 ), 0.3);
        robot.Stop();
        sleep(100);

        hooks.UnLatched();
        sleep(1000);

        robot.RobotMoveX(new Waypoint(-70, 0, 0), 0.3);
        robot.Stop();
        sleep(100);

        robot.RobotMoveY(new Waypoint(0, 80, 0), 0.3);
        robot.Stop();
        sleep(100);

        robot.RobotMoveX(new Waypoint(75, 0, 0), 0.3);
        robot.Stop();
        sleep(100);

        robot.RobotMoveY(new Waypoint(0, -70, 0), 0.4);
        robot.Stop();
        sleep(100);

        robot.RobotMoveY(new Waypoint(0, 10, 0), 0.4);
        robot.Stop();
        sleep(100);

        robot.RobotMoveX(new Waypoint(-125 , 0, 0), 0.3);
        robot.Stop();
        sleep(100);
    }


}
