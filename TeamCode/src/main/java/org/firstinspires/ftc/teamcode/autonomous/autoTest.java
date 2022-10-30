package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Autonomous(name="AutonomousTest", group="--")
//@Disabled
public class autoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx FRM = null; // V2
    private DcMotorEx BRM = null; // V4
    private DcMotorEx FLM = null; // V1
    private DcMotorEx BLM = null; // V3
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;


    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    int target;
    // REV Motors TICK COUNT = 28 ticks




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime holdTimer = new ElapsedTime();

        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");
        // rightIntake = hardwareMap.get(CRServo.class,"WheelRight");
        // leftIntake = hardwareMap.get(CRServo.class,"WheelLeft");

        waitForStart();

        BRM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);
        // leftIntake.setDirection(CRServo.Direction.REVERSE);

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 5000 - first test 228 cm
        // 2000 - 92.4 cm, 91.5 cm, 90.1 cm tilted left, 91.3 cm tilted left
        for (int i = 0; i < 10; i++)
        {
            square();
        }
        // square
    }

    public void square() {
        runStraight(60);
        sleep(300);
        strafe(60);
        sleep(300);
        runStraight(-60);
        sleep(300);
        strafe(-60);
    }

    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * -35.6);
    }// calculation

    public void strafe(int centimeters) {
        int x_ticks = CMtoTicks(centimeters);

        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRM.setTargetPosition(-x_ticks);
        BRM.setTargetPosition(x_ticks);
        FLM.setTargetPosition(x_ticks);
        BLM.setTargetPosition(-x_ticks);

        FRM.setPower(.5);
        BRM.setPower(.5);
        FLM.setPower(.5);
        BLM.setPower(.5);

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (FRM.isBusy() && BRM.isBusy() && FLM.isBusy() && BLM.isBusy()) {

        }

        FRM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BLM.setPower(0);

    }

    public void runStraight(double centimeters) {
        int ticks = CMtoTicks(centimeters);

        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRM.setTargetPosition(ticks);
        BRM.setTargetPosition(ticks);
        FLM.setTargetPosition(ticks);
        BLM.setTargetPosition(ticks);

        /*
        direction = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) + (Math.PI)/2 - gyroheading;
        speed = Math.min(1.0, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));

        FLPower = Math.max(-1,(speed * Math.sin(direction + Math.PI / 4.0)  - directionMultiplier*gamepad1.right_stick_x));
        FRPower = -Math.max(-1, (speed * Math.cos(direction + Math.PI / 4.0) - directionMultiplier*gamepad1.right_stick_x));
        BLPower = -Math.max(-1, (speed * Math.cos(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x));
        BRPower = Math.max(-1, (speed * Math.sin(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x));
        */

        FRM.setPower(.5);
        BRM.setPower(.5);
        FLM.setPower(.5);
        BLM.setPower(.5);

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (FRM.isBusy() && BRM.isBusy() && FLM.isBusy() && BLM.isBusy()) {

        }

        FRM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BLM.setPower(0);

    }
}