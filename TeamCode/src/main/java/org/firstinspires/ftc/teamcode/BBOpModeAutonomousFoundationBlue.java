package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-Foundation-Blue", group="BB")
public class BBOpModeAutonomousFoundationBlue extends LinearOpMode
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
        robot.LocalizerUpdate();
        Waypoint way1 = new Waypoint(0, 55.0, 0);
        Waypoint way2 = new Waypoint(20, 0, 0);

        //while(this.opModeIsActive()){
            robot.RobotMoveY(way1, 0.2);
            robot.RobotMoveX(way2, 0.2);

          //  if (robot.pose().distance(way1) < way1.followDistance){
               // robot.Stop();
               // break;
          //  }
         //   robot.LocalizerUpdate();
       // }



    }


}
