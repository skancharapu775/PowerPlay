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

@Autonomous(name="accelerometerPositioning", group="--")
//@Disabled
public class accelerometerPositioning extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx FRM = null;
    private DcMotorEx BRM = null;
    private DcMotorEx FLM = null;
    private DcMotorEx BLM = null;


    // distance trackers
    double acceleration_magnitude = 0;
    double velocity = 0;
    double distance = 0;

    double FRPower, BRPower, FLPower, BLPower;
    double directionMultiplier = 0.5;
    int target;

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

        waitForStart();

        BRM.setDirection(DcMotorEx.Direction.REVERSE);
        FRM.setDirection(DcMotorEx.Direction.REVERSE);

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void distance(double angle, double distance) {
        double total_distance = 0;
        double previousVelocity = 0;
        double currentVelocity = 0;
        double previousTime = 0;
        double currentTime = 0;
        double temp;


        double direction = ((Math.PI)/180)*angle;
        double acceleration = 0;

        // start time

        FLPower = (0.8 * Math.sin(direction + Math.PI / 4.0)  - directionMultiplier*gamepad1.right_stick_x) * -1;
        FRPower = -(0.8 * Math.cos(direction + Math.PI / 4.0) - directionMultiplier*gamepad1.right_stick_x) * -1;
        BLPower = -(0.8 * Math.cos(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x) * -1;
        BRPower = (0.8 * Math.sin(direction + Math.PI / 4.0) + directionMultiplier*gamepad1.right_stick_x) * -1;

        while (total_distance < distance) {
            acceleration = 0; // replace with imu value
            temp = currentVelocity;
            currentVelocity = previousVelocity + acceleration;
            previousVelocity = currentVelocity;

            previousTime = currentTime;
            currentTime = 0; // replace with elapsed time value

            total_distance += currentVelocity * (currentTime - previousTime);
        }
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

}
