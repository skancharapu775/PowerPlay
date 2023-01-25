package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="basicMecanumDrive", group="--")
//@Disabled
public class basicMecanumDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;
    private DcMotor slide = null; // 288 ticks per rotation
    private CRServo rightIntake = null;
    private CRServo leftIntake = null; // 288 ticks per rotation

    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    boolean slugMode = false;
    double slugMultiplier = 0.33;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FRM = hardwareMap.get(DcMotor.class, "frontRight");
        BRM = hardwareMap.get(DcMotor.class, "backRight");
        FLM = hardwareMap.get(DcMotor.class, "frontLeft");
        BLM = hardwareMap.get(DcMotor.class, "backLeft");
        rightIntake = hardwareMap.get(CRServo.class,"WheelRight");
        leftIntake = hardwareMap.get(CRServo.class,"WheelLeft");

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
        leftIntake.setDirection(CRServo.Direction.REVERSE); // change if needed

        boolean output = false;
        boolean intake = false;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //if (gamepad1.a)
            //{
            //  slugMode = !slugMode;
            //}

            // multiply by -1 to change front of robot
            FRPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - directionMultiplier*gamepad1.right_stick_x) * -1;
            BRPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - directionMultiplier*gamepad1.right_stick_x) * -1;
            FLPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + directionMultiplier*gamepad1.right_stick_x) * -1;
            BLPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + directionMultiplier*gamepad1.right_stick_x) * -1;

            //if (slugMode)
            //{
            // FRPower = FRPower * slugMultiplier;
            //BRPower = BRPower * slugMultiplier;
            //FLPower = FLPower * slugMultiplier;
            //BLPower = BLPower * slugMultiplier;
            //}

            FRM.setPower(FRPower);
            BRM.setPower(BRPower);
            FLM.setPower(FLPower);
            BLM.setPower(BLPower);

            /*if (gamepad1.left_trigger > gamepad1.right_trigger) {
                intake = false;
                output = true;
            }
            else if (gamepad1.right_trigger > gamepad1.left_trigger) {
                intake = true;
                output = false;
            }
            else if (gamepad1.right_trigger > -.2 && gamepad1.left_trigger > -.2) {
                intake = false;
                output = false;
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }
//            else {
//                intake = false;
//                output = false;
//            }

            double intakePower = gamepad1.right_trigger;
            double outputPower = gamepad1.left_trigger;

            if (intake){
                rightIntake.setPower(intakePower);
                leftIntake.setPower(intakePower);
            }
            if (output) {
                rightIntake.setPower(-outputPower);
                leftIntake.setPower(-outputPower);
            }*/

            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.update();

        }
    }
}