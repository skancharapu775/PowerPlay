package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name="AutonomousTest", group="--")
//@Disabled
public class mainAutonomous extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx FRM = null; // V2
    private DcMotorEx BRM = null; // V4
    private DcMotorEx FLM = null; // V1
    private DcMotorEx BLM = null; // V3
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;

    // case (Left perspective looking at field from starting point)

    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    // REV Motors TICK COUNT = 28 ticks

    double speed = 0.5;

    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

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

        waitForStart();

        BRM.setDirection(DcMotorEx.Direction.REVERSE);
        FRM.setDirection(DcMotorEx.Direction.REVERSE);
        // leftIntake.setDirection(CRServo.Direction.REVERSE);

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        oneLeft();
    }

    private void oneLeft() {
        runStraight(6, 90);
        turnToAngle(-90);
        runStraight(60, 90);
        turnToAngle(90);
        runStraight(70, 90);
        turnToAngle(45);
        runStraight(8, 90);
        sleep(2000);
        runStraight(4, -90);
    }

    private void twoLeft() {
        runStraight(132, 90);
        turnToAngle(45);
        runStraight(8, 90);
        sleep(2000);
        runStraight(4, -90);
    }

    private void threeLeft() {
        runStraight(6, 90);
        turnToAngle(90);
        runStraight(60, 90);
        turnToAngle(-90);
        runStraight(70, 90);
        turnToAngle(45);
        runStraight(8, 90);
        sleep(2000);
        runStraight(4, -90);

    }

    private void oneRight() {

    }

    private void twoRight() {

    }

    private void threeRight() {

    }

    private double getAngle() {
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


    public int CMtoTicks(double DistanceCM){

        return (int) (DistanceCM * -23.7671);
    }// calculation

    public void runStraight(double centimeters, double direction) {
        int ticks = CMtoTicks(centimeters);

        FRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FLPower = (speed * Math.sin(direction*(Math.PI/180) + Math.PI / 4.0));
        FRPower = -(speed * Math.cos(direction*(Math.PI/180) + Math.PI / 4.0));
        BLPower = -(speed * Math.cos(direction*(Math.PI/180) + Math.PI / 4.0));
        BRPower = (speed * Math.sin(direction*(Math.PI/180) + Math.PI / 4.0));

//        double FLBRTickMultiplier = FLPower/FRPower;

        if(FRPower<0){
            FRM.setTargetPosition(-ticks);
        }
        else{
            FRM.setTargetPosition(ticks);
        }

        if(FLPower<0){
            FLM.setTargetPosition(-ticks);
        }
        else{
            FLM.setTargetPosition(ticks);
        }

        if(BRPower<0){
            BRM.setTargetPosition(-ticks);
        }
        else{
            BRM.setTargetPosition(ticks);
        }

        if(BLPower<0){
            BLM.setTargetPosition(-ticks);
        }
        else{
            BLM.setTargetPosition(ticks);
        }

        FLM.setPower(FLPower);
        FRM.setPower(FRPower);
        BLM.setPower(BLPower);
        BRM.setPower(BRPower);

        FRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("FRM", FRPower);
        telemetry.addData("FLM",FLPower);
        telemetry.addData("BRM", BRPower);
        telemetry.addData("BLM", BLPower);
        telemetry.update();


        while (FRM.isBusy() && BRM.isBusy() && FLM.isBusy() && BLM.isBusy()) {
//
//            telemetry.addData("FRM", FRM.getCurrentPosition() );
//            telemetry.addData("FLM", FLM.getCurrentPosition() );
//            telemetry.addData("BRM", BRM.getCurrentPosition() );
//            telemetry.addData("BLM", BLM.getCurrentPosition() );

            telemetry.update();
        }

        FRM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BLM.setPower(0);
    }

    public void turnToAngle(double degrees){
        degrees = degrees * -1;
        double currentAngle = getAngle();
        double pow;
        double offset = currentAngle+degrees - getAngle();
        FRM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        while (getAngle()>currentAngle+degrees+1 || getAngle()<currentAngle+degrees-1){
            pow = (offset / 30);
            pow = Math.max(Math.min(pow, 1), -1);
            if(getAngle()>currentAngle+degrees+1){
                FLM.setPower(pow);
                BLM.setPower(pow);
                FRM.setPower(-pow);
                BRM.setPower(-pow);
            }
            if(getAngle()<(currentAngle+degrees-1)
            ){
                FLM.setPower(-pow);
                BLM.setPower(-pow);
                FRM.setPower(pow);
                BRM.setPower(pow);
            }
        }
    }
}