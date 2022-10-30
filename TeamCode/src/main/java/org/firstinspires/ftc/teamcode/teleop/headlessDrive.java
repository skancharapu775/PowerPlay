package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="headlessDrive", group="--")
//@Disabled
public class headlessDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;
    private DcMotor slide = null; // 288 ticks per rotation
    private CRServo rightIntake = null;
    private CRServo leftIntake = null;


    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    double intakePower = 0.8;
    double outtakePower = 0.5;


    // Setting up Slug Mode Parameters
    boolean slugMode = false;
    double slugMultiplier = 0.33;

    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        FRM = hardwareMap.get(DcMotor.class, "frontRight");
        BRM = hardwareMap.get(DcMotor.class, "backRight");
        FLM = hardwareMap.get(DcMotor.class, "frontLeft");
        BLM = hardwareMap.get(DcMotor.class, "backLeft");
        slide = hardwareMap.get(DcMotor.class, "liftMotor");
        rightIntake = hardwareMap.get(CRServo.class,"leftWheel"); // configure these two
        leftIntake = hardwareMap.get(CRServo.class,"rightWheel");

        //GamePads to save previous state of gamepad for toggling slug mode
        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);

        // Setting parameters for imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //direction for headless mode
        double direction;
        double speed;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try{
                //setting previous state of gamepad1 and current position for later use toggling slug mode
                previousGamePad1.copy(currentGamePad1);
                currentGamePad1.copy(gamepad1);
            }
            catch(RobotCoreException e){
            }

            // button a to toggle slug mode
            if (currentGamePad1.a && !previousGamePad1.a)
            {
                slugMode = !slugMode;
            }

            // linear slide
            if (currentGamePad1.right_trigger > 0)
            {
                slide.setPower(-currentGamePad1.right_trigger);
            }
            else if (currentGamePad1.left_trigger > 0)
            {
                slide.setPower(currentGamePad1.left_trigger);
            }
            else
            {
                slide.setPower(0);
            }

            // intake
            if (currentGamePad1.right_bumper && !currentGamePad1.left_bumper)
            {
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);
            }
            else if (currentGamePad1.left_bumper && !currentGamePad1.right_bumper)
            {
                leftIntake.setPower(-intakePower);
                rightIntake.setPower(-intakePower);
            }


            // setting direction, atan2 gives theta in polar coordinates
            direction = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - (getAngle()*(Math.PI/180)-(Math.PI/2));
            speed = Math.min(1.0, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));

//          Math.min is to make sure the powers aren't set to anything more than one for now
//          Dividing by 28 to get a number between -1 and 1 (power percentage instead of ticks/sec)
            FLPower = (speed * Math.sin(direction + Math.PI / 4.0)  - directionMultiplier*gamepad1.right_stick_x);
            FRPower = -(speed * Math.cos(direction + Math.PI / 4.0) - directionMultiplier*gamepad1.right_stick_x);
            BLPower = -(speed * Math.cos(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x);
            BRPower = (speed * Math.sin(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x);

            if (slugMode){
                FRPower = FRPower * slugMultiplier;
                BRPower = BRPower * slugMultiplier;
                FLPower = FLPower * slugMultiplier;
                BLPower = BLPower * slugMultiplier;
            }

            FRM.setPower(FRPower);
            BRM.setPower(BRPower);
            FLM.setPower(FLPower);
            BLM.setPower(BLPower);


            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.addData("Direction", getAngle());
            telemetry.update();

        }
    }

    private double getAngle()
    {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
           We have to process the angle because the imu works in euler angles so the Z axis is
           returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
           180 degrees. We detect this transition and track the total cumulative angle of rotation. */

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

}