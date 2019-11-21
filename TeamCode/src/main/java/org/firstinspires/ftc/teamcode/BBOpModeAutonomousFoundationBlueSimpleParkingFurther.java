package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-Foundation-Blue-Simple-Further", group="BB")
public class BBOpModeAutonomousFoundationBlueSimpleParkingFurther extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.LocalizerUpdate();

        Waypoint way1 = new Waypoint(0, 55.0, 0);
         //while(this.opModeIsActive()){
        robot.RobotMoveY(way1, 0.3);
        robot.Stop();
        Waypoint way2 = new Waypoint(20, 0, 0);
        robot.RobotMoveX(way2, 0.3);



    }


}
