package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumDrive", group="--")
//@Disabled
public class mecanumDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;

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

            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.update();

        }
    }
}