
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.common.math.TimePose;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Locale;


public class BBSRobot {

    private BBHooks _hooks = new BBHooks();
    private BBIntake _intakeMotor = new BBIntake();
    private BBLift _lift = new BBLift();

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;

    private ExpansionHubEx chassisHub1;
    private ExpansionHubEx chassisHub2;

    public TwoWheelTrackingLocalizer localizer;
    public RevBulkData lastChassis1Read;
    public RevBulkData lastChassis2Read;
    public double lastHeading;
    private double headingOffset;

    private Telemetry telemetry;

    private LinearOpMode _mode;

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;


    PIDController           pidRotate, pidDrive;




    public void init(HardwareMap hwmap, Telemetry tele, LinearOpMode mode)
    {


        telemetry = tele;
        _intakeMotor.init(hwmap);
        _lift.init(hwmap, tele, mode);
        _hooks.init(hwmap);
        _mode = mode;


        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.01, .00006, 0);

        leftFront = hwmap.get(DcMotor.class, "left_front");
        rightFront = hwmap.get(DcMotor.class, "right_front");

        rightRear = hwmap.get(DcMotor.class, "right_rear");
        leftRear = hwmap.get(DcMotor.class, "left_rear");

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        imu = hwmap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        // remap axis
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;


        runtime.reset();
        Pose start = new Pose(0,0,0);
        TimePose startPose4D = new TimePose(start, System.currentTimeMillis());
        localizer = new TwoWheelTrackingLocalizer(0, 1, startPose4D);
        this.lastHeading = 0;

        chassisHub1 = hwmap.get(ExpansionHubEx.class, "Expansion Hub 1");
        chassisHub2 = hwmap.get(ExpansionHubEx.class, "Expansion Hub 2");
    }

    public void LocalizerUpdate(){
        this.lastChassis1Read = chassisHub1.getBulkInputData();
        this.lastChassis2Read = chassisHub2.getBulkInputData();
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(lastChassis1Read, lastChassis2Read, lastHeading);
    }

    public void Move(Gamepad gp1, Gamepad gp2){



        this.lastChassis1Read = chassisHub1.getBulkInputData();
        this.lastChassis2Read = chassisHub2.getBulkInputData();
        this.lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        localizer.update(lastChassis1Read, lastChassis2Read, lastHeading);

        telemetry.addData("X:", String.format("%.1f", localizer.x()));
        telemetry.addData("Y:",String.format("%.1f", localizer.y()));
        telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));
        telemetry.update();

        //Slow the robot down if the left trigger is used.
        double slowScale = ((1 - gp1.left_trigger) * 0.7 + 0.4);

        if(gp1.left_trigger == 0){
            slowScale = 0.8;
        }
        
        double leftX = MecanumUtil.deadZone(gp1.left_stick_x, 0.05) * slowScale;
        double leftY = MecanumUtil.deadZone(gp1.left_stick_y, 0.05) * slowScale;
        
        //work out the angle in which the joystick is held
        double angle = Math.atan2(leftY, leftX) + Math.PI / 2;

        //work out the magnitude (or hypotenuse) of our stick position
        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        //make sure its between zero and one
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn - i.e. work out what our strafing should look like
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(gp1.right_stick_x, 0.05), 2),
                gp1.right_stick_x) * slowScale;

        //pass in the angle to move, the speed, then the strafing amount
        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
        setPowers(powers);

        if(gp1.right_bumper || gp2.right_bumper){
            _intakeMotor.Start();
        }

        if(!gp1.right_bumper || !gp2.right_bumper){
            _intakeMotor.Stop();
        }

        if(gp1.left_bumper || gp2.left_bumper){
            _intakeMotor.Reverse();
        }

        if(!gp1.left_bumper || !gp2.left_bumper){
            _intakeMotor.Stop();
        }


        if (gp1.left_stick_button){
            _hooks.ToggleHook();
            //Add delay
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }

        if(gp1.dpad_up || gp2.dpad_up) {
            _lift.DropOff();
        }
        else if(gp1.dpad_down || gp2.dpad_down){
            _lift.Home();
        }
        else{
            _lift.Off();
        }

        if(gp1.y || gp2.y){
            _lift.GoToHome();
        }
        if(gp1.x || gp2.x){
            _lift.LoadBrick();
        }

        if(gp1.a || gp2.a){
            _lift.Grip();
        }

        if(gp1.b || gp2.b){
            _lift.Release();
        }

       

    }

    private void setPowers(MecanumPowers powers)
    {
        leftFront.setPower(powers.frontLeft);
        rightFront.setPower(powers.frontRight);
        leftRear.setPower(powers.backLeft);
        rightRear.setPower(powers.backRight);


    }


    public static Pose SLIP_DISTANCES = new Pose(22, 5, Math.toRadians(30));
    public static double MIN_TRANSLATION_POWERS = 0.2;
    public static double CUTOFF_MOTOR_POWER = 0.05;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(6, 6, Math.PI/2);
    public static Pose FINE_REDUCTION_DISTANCES = new Pose(3, 3, Math.PI);

     //test function that can use a waypoint in absoulte co-ordinates to move.
     public void RobotMove(Waypoint target, double movementSpeed) {

         telemetry.addData("X:", String.format("%.1f", localizer.x()));
         telemetry.addData("Y:",String.format("%.1f", localizer.y()));
         telemetry.addData("H:",String.format("%.1f", MathUtil.angleWrap(Math.toDegrees(localizer.h()))));


        //we can use our localiser to move to the position that we want.
         double distance = target.minus(localizer.pose()).radius();
         double relAngle = localizer.pose().minus(target).atan() - localizer.pose().heading;
         double relX = distance * Math.cos(relAngle);
         double relY = distance * Math.sin(relAngle);

         Pose translationPowers = new Pose(relX, relY, 0).scale(movementSpeed);

         Pose reductionDistances = FINE_REDUCTION_DISTANCES;
         translationPowers.x /= reductionDistances.x;
         translationPowers.y /= reductionDistances.y;

         // Ensure we're moving x/y a little unless we're stopped
         if (translationPowers.radius() < MIN_TRANSLATION_POWERS) {
             translationPowers.scale(MIN_TRANSLATION_POWERS / translationPowers.radius());
         }

         telemetry.addData("distance:", String.format("%.1f", distance));
         telemetry.addData("relAngle:", String.format("%.1f", relAngle));

         double forwardAngle = target.minus(localizer.pose()).atan();
         double backwardAngle = forwardAngle + Math.PI;
         double angleToForward = MathUtil.angleWrap(forwardAngle - localizer.pose().heading);
         double angleToBackward = MathUtil.angleWrap(backwardAngle - localizer.pose().heading);
         double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
         double desiredAngle = autoAngle;

         // We'll do a quadratic decay to zero to only correct for really big heading errors
         double angleToTarget = MathUtil.angleWrap(desiredAngle - localizer.pose().heading);
         translationPowers.heading = angleToTarget / reductionDistances.heading;

         telemetry.addData("angleToTarget:", String.format("%.1f", angleToTarget));

         telemetry.addData("autoAngle:", String.format("%.1f", autoAngle));
         telemetry.addData("t1x:", String.format("%.1f", translationPowers.x));
         telemetry.addData("t1y:", String.format("%.1f", translationPowers.y));
         telemetry.addData("t1h:", String.format("%.1f", translationPowers.heading));
         //power the wheels
         //setPowers(new MecanumPowers(translationPowers));

         telemetry.update();
    }

    //move the robot in the Y plane
    public void RobotMoveY(Waypoint target, double movementSpeed) {

        //reset the encoders
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double angle = 0;

        if(target.y < 0){
            
            double y = MecanumUtil.deadZone(1.0, 0.05) * 0.3;
            angle = Math.atan2(y, 0) + Math.PI / 2;
        }

        resetAngle();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, movementSpeed);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        LocalizerUpdate();
        while(_mode.opModeIsActive() &&  Math.abs(localizer.y()) < Math.abs(target.y)) {
            double correction = pidDrive.performPID(getAngle());
            LocalizerUpdate();
            MecanumPowers powers = MecanumUtil.powersFromAngle(angle - correction, movementSpeed, 0);

            //use a pid
            telemetry.addLine("PID");
            telemetry.addData("Correction", correction);
            telemetry.addData("Angle", getAngle());
            telemetry.update();

            setPowers(powers);

        }
    }

    

    public void RobotMoveX(Waypoint target, double movementSpeed) {

        //reset the encoders
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidDrive.setOutputRange(0, 0.05);
        pidDrive.setInputRange(-90,90);
        pidDrive.setSetpoint(0);
        pidDrive.enable();
        getAngle();
      
        LocalizerUpdate();
        while(_mode.opModeIsActive() && Math.abs(localizer.x()) < Math.abs(target.x)) {
            LocalizerUpdate();

            double slowScale = 0.3;

            double direction = 1;

            if(target.x  < 0){
                direction = -1;
            }

            // strafing speed
            double turn = Math.copySign(
                    Math.pow(MecanumUtil.deadZone(direction, 0.05), 2),
                    direction) * slowScale;

            MecanumPowers powers = MecanumUtil.powersFromAngle(0, 0, turn);


            double correction = pidDrive.performPID(getRightAngle());

            telemetry.addData("Strafe correction", correction);
            telemetry.addData("angle", getRightAngle());
            telemetry.update();

            //our robot needs a boost on the back wheels
            //a little bit of friction is present.
            if(target.x  < 0) {
                powers.backRight -= correction;
                powers.backLeft += correction;
            }
            else{

                powers.backRight -= correction;
                powers.backLeft += correction;
            }
            setPowers(powers);
            
          
        }
    }

    public Pose pose(){
         return localizer.pose();
    }

    public void Stop(){
         setPowers(new MecanumPowers(0,0,0,0));
    }

    public void moveBackwards(int centimetres, double speed){

    }

    public void moveForward()
    {
        double slowScale = 0.18;
        double leftX = MecanumUtil.deadZone(0, 0.05) * slowScale;
        double leftY = MecanumUtil.deadZone(-1, 0.05) * slowScale;
        double angle = Math.atan2(leftY, leftX) + Math.PI / 2;

        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(0, 0.05), 2),
                0) * slowScale;


        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, turn);
        setPowers(powers);
    }

    //use the gyro to do a left point turn
    //uses a PID to slow it down as it approaches the angle
    public void turnLeft(double degrees, double speed){

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        runtime.reset();

        resetAngle();

        telemetry.addData("Angle:", getAngle());
        telemetry.update();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, speed);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        left(speed);
        _mode.sleep(100);
        do {

            double power = pidRotate.performPID(Math.abs(getAngle()));

            if(power > 0){
                left(Math.abs(power));
            }else{
                right(Math.abs(power));
            }

            LocalizerUpdate();

         

        } while(runtime.seconds() < 5 && _mode.opModeIsActive() && !pidRotate.onTarget());


    }

    //use the gyro to do a right point turn
    //uses a PID to slow it down as it approaches the angle
    private void right(double speed){
        double slowScale = speed;

        double leftX = MecanumUtil.deadZone(1, 0.05) * slowScale; // make the magnitude position less (with the speed)
        double leftY = 0; //nothing on the Y for a turn
        double angle = Math.atan2(leftY, leftX) + Math.PI / 2;

        //calculate the magnitude
        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        //ensure its between zero to one
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn - return a positive number
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(1, 0.05), 2),
                1) * slowScale;


        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, 0);
        setPowers(powers);
    }

    private void left(double speed)
    {
        double slowScale = speed;

        double leftX = MecanumUtil.deadZone(-1, 0.05) * slowScale;
        double leftY = 0;
        double angle = Math.atan2(leftY, leftX) + Math.PI / 2;

        double driveScale = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        driveScale = Range.clip(driveScale, 0, 1);

        // Exponentiate our turn
        double turn = Math.copySign(
                Math.pow(MecanumUtil.deadZone(-1, 0.05), 2),
                1) * slowScale;


        MecanumPowers powers = MecanumUtil.powersFromAngle(angle, driveScale, 0);
        setPowers(powers);
    }

    public void turnRight(double degrees, double speed)
    {
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        runtime.reset();

        resetAngle();

        telemetry.addData("Angle:", getAngle());
        telemetry.update();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, speed);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        right(speed);
        _mode.sleep(100);
        do {

            double power = pidRotate.performPID(Math.abs(getAngle()));

            if(power > 0){
                right(Math.abs(power));
            }else{
                left(Math.abs(power));
            }

            LocalizerUpdate();

           

        } while(runtime.seconds() < 5 && _mode.opModeIsActive() && !pidRotate.onTarget());


    }
    
    //store values for the location - used for gyro turning
    Orientation             lastAngles = new Orientation();
    Orientation             lastRightAngles = new Orientation();
    double                  globalAngle, globalRightAngle;

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
        globalRightAngle = 0;
    }

    private double getRightAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.thirdAngle - lastAngles.thirdAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalRightAngle += deltaAngle;

        lastAngles = angles;

        return globalRightAngle;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    public void SkyHookOn(){
         _hooks.SkyHookOn();
    }

    public void SkyHookOff(){
        _hooks.SkyHookOff();
    }

}
