package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="AutonomousTest1", group="--")
//@Disabled
public class autoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null; // V2
    private DcMotor BRM = null; // V4
    private DcMotor FLM = null; // V1
    private DcMotor BLM = null; // V3
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;


    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    int one_rotation = 28;
    int target;
    // REV Motors TICK COUNT = 28 ticks




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

        waitForStart();

        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(CRServo.Direction.REVERSE);

        runStraight(2*one_rotation);

    }



    public void runStraight(int ticks) {
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

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        
        FRM.setVelocity(one_rotation);
        BRM.setVelocity(one_rotation/2);
        FLM.setVelocity(one_rotation/2);
        BLM.setVelocity(one_rotation/2);


    }
}