package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

import java.util.List;


@Autonomous(name="Auto-SkyBridge-Blue", group="BB")
public class BBAutoBlueSkystoneBridgePark extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();

    private BBIntake intake = new BBIntake();
    private BBVision _vision = new BBVision();

    @Override
    public void runOpMode() {

        // initialising components
        robot.init(hardwareMap, telemetry, this);
        intake.init(hardwareMap);
        _vision.setUp(telemetry, hardwareMap);

        // telemetry to show robot is ready to start
        telemetry.addData("Status", "Starting Auto");
        telemetry.update();

        // wait for the game to start
        waitForStart();

        int movesForward = 0;
        boolean foundStone = false;

        runtime.reset();

        robot.RobotMoveX(new Waypoint(50,0,0),0.2);
        robot.Stop();

        List<Recognition> targets = _vision.visionFeedback(telemetry);

        Recognition foundTarget = null;

        while(runtime.seconds() < 15 && this.opModeIsActive() && foundStone == false && movesForward < 5) {
            if (targets != null && targets.size() > 0) {
                // found something
                for (int count = 0; count < targets.size(); count++) {

                    telemetry.addLine(targets.get(count).getLabel());

                    if (targets.get(count).getLabel() == "Skystone") {
                        foundTarget = targets.get(count);
                        telemetry.addLine("SKYSTONE FOUND");
                        telemetry.addData("Pos", foundTarget.getLeft());
                        telemetry.addData("Right", foundTarget.getRight());
                        telemetry.addData("Left", foundTarget.getLeft());
                        telemetry.update();
                        foundStone = true;
                        robot.Stop();
                        break;
                    }
                }

                if (foundStone) {
                    break;
                }

                telemetry.addLine("Moving forward now");
                telemetry.update();

                robot.RobotMoveY(new Waypoint(0,-10,0), 0.2);
                robot.Stop();

                TimeElapsedPause(500);

                movesForward++;
            }

            else {
                telemetry.addLine("no brick found yet, keep moving");
                telemetry.addData("moves:", movesForward);
                telemetry.update();

                // move forward until a block is found
                robot.RobotMoveY(new Waypoint(0,-5,0), 0.2);
                robot.Stop();

                TimeElapsedPause(1000);
                movesForward++;
            }

            targets = _vision.visionFeedback(telemetry);
            telemetry.addData("movesForwards:", movesForward);
        }

        if (foundStone) {

            //we need to move to the stone (ie. left or right)
            telemetry.addLine("Moving towards the skystones");
            //use the left / right position to determine how far to move
            telemetry.addData("L", foundTarget.getLeft());
            telemetry.addData("R", foundTarget.getRight());
            telemetry.addData("A", foundTarget.estimateAngleToObject(AngleUnit.DEGREES));

            double foundAngle = foundTarget.estimateAngleToObject(AngleUnit.DEGREES);

            telemetry.update();

            while (this.opModeIsActive() && Math.abs(foundAngle) > 1) {
                foundAngle = foundTarget.estimateAngleToObject(AngleUnit.DEGREES);

                telemetry.addLine("Moving to SKY stone");
                telemetry.addData("Angle", foundAngle);
                telemetry.update();

                if (foundAngle > 2) {
                    robot.RobotMoveY(new Waypoint(0, -1, 0), 0.18);
                } else {
                    robot.RobotMoveY(new Waypoint(0, 1, 0), 0.18);
                }

                robot.Stop();
                TimeElapsedPause(100);

                targets = _vision.visionFeedback(telemetry);

                if (targets == null || targets.size() == 0) {
                    break;
                }

                for (int count = 0; count < targets.size(); count++) {
                    if (targets.get(count).getLabel() == "Skystone") {
                        foundTarget = targets.get(count);
                        break;
                    }
                }
            }

            // code for once the skystone has been found
            robot.RobotMoveX(new Waypoint(30,0,0), 0.2);
            robot.Stop();
            TimeElapsedPause(500);
            robot.SkyHookOn();
            //TimeElapsedPause(700);
            sleep(2000);
            robot.RobotMoveX(new Waypoint(-5,0,0),0.2);
            TimeElapsedPause(1000);
            robot.RobotMoveY(new Waypoint(0,(movesForward * 10 + 90), 0), 0.4);
            robot.Stop();
            TimeElapsedPause(300);

            robot.SkyHookOff();
            TimeElapsedPause(100);
            robot.RobotMoveY(new Waypoint(0,-40,0), 0.2);
            robot.Stop();
        }

        else {
            robot.Stop();
            telemetry.addLine("No stone has been found - parking now");
            // TODO: look at the localiser and work out how far back we need to go to park

            //robot.RobotMoveY(new Waypoint(0,-8,0), 0.2);
            robot.RobotMoveX(new Waypoint(30,0,0), 0.2);
            robot.Stop();
            TimeElapsedPause(500);
            robot.SkyHookOn();
            //TimeElapsedPause(700);
            sleep(2000);
            robot.RobotMoveX(new Waypoint(-5,0,0),0.2);
            TimeElapsedPause(1000);
            robot.RobotMoveY(new Waypoint(0,170, 0),0.4);
            robot.Stop();
            TimeElapsedPause(300);

            robot.SkyHookOff();
            TimeElapsedPause(100);
            robot.RobotMoveY(new Waypoint(0,-40,0), 0.2);
            robot.Stop();
        }

        _vision.cleanUp();
    }

    public void TimeElapsedPause(double ms) {
        ElapsedTime timer = new ElapsedTime();

        while(timer.milliseconds() < ms) {
            idle();
        }
    }
}

