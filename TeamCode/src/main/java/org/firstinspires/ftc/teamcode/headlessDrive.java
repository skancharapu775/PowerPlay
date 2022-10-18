package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="headlessMode", group="--")
//@Disabled
public class headlessDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;
    private GyroSensor gyroscope = null;

    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    boolean slugMode = false;
    double slugMultiplier = 0.33;
    double gyroheading = 0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        FRM = hardwareMap.get(DcMotor.class, "frontRight");
        BRM = hardwareMap.get(DcMotor.class, "backRight");
        FLM = hardwareMap.get(DcMotor.class, "frontLeft");
        BLM = hardwareMap.get(DcMotor.class, "backLeft");
        gyroscope = hardwareMap.get(GyroSensor.class, "");


        //GamePads to save previous state of gamepad
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

        BRM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);

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

            if (currentGamePad1.a && !previousGamePad1.a)
            {
                slugMode = !slugMode;
            }

            // assume heading is positive gyro change
            //setting direction, atan2 gives theta from polar coordinates
            direction = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) + (Math.PI)/2 - gyroheading;
            speed = Math.min(1.0, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));

            telemetry.addData("direction", direction);
            telemetry.addData("speed", speed);

//            Math.min is to make sure the powers aren't set to anything more than one for now
//            Dividing by 28 to get a number between -1 and 1 (power percentage instead of ticks/sec)
            FLPower = Math.max(-1,(speed * Math.sin(direction + Math.PI / 4.0)  - directionMultiplier*gamepad1.right_stick_x));
            FRPower = -Math.max(-1, (speed * Math.cos(direction + Math.PI / 4.0) - directionMultiplier*gamepad1.right_stick_x));
            BLPower = -Math.max(-1, (speed * Math.cos(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x));
            BRPower = Math.max(-1, (speed * Math.sin(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x));
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
            telemetry.update();

        }
    }
}