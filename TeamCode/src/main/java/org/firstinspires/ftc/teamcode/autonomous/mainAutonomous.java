package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.HashMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name="MainAutonomous", group="--")
//@Disabled
public class mainAutonomous extends LinearOpMode {
    // Declare OpMode members.
    private DcMotorEx FRM = null; // V2
    private DcMotorEx BRM = null; // V4
    private DcMotorEx FLM = null; // V1
    private DcMotorEx BLM = null; // V3
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;

    // main timer
    private ElapsedTime runtime = new ElapsedTime();

    // time to press start after initialization
    double starting_time = 0;

    //power variables
    double FRPower, BRPower, FLPower, BLPower;
    double speed = 0.5;

    double turningPower = 0.3;
    double errorMargin = 0.5; // degrees
    double drive = 1; // auto = -1, drive = 1
    double theoreticalAngle;

    // object detection cases (Left perspective looking at field from starting point)
    double left_side = 0;
    double right_side = 0;
    double sleeveNum;
    private static final String VUFORIA_KEY =
            "AbIGMsX/////AAABmdF2oR5gBUIhgUBTLH8qL2ko4GIwZFzoIejoc3ZRmUqg5cCR4aZXR4E1Y2tl4U++tqyvj/TKTbG22Pwj96fJhnHU+fb6MzWG6kAMhzo1Leg/fVhBgFk2Xb461uzYZvXr2PKJQAX7MQiyNblyArGGEjZL0oZ9129UjgvUMUP3oVipQiuVchoOG4YV3xZrlYWxankBupy326b96vLO2vbPOunRHWQmsTeOmdYB8DNqrAnxLCkYpa/YWwyYx1Es+bN8Je4pXbJuYjSQxYR36mkjpkbIardjQqte9CDxxMyD8CgUDMrf7L9Z9gaTM+vC1Tv8I6Ei0m1NfGMWsG/RT0fr1om/pVAkC61+j8IRb+JCYpVB";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "InstinctSleeveDetection3000steps.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String[] LABELS = {
            "Square", // 1
            "Triangle", // 2
            "Circle" // 3
    };

    // Gamepad previousGamePad1 = new Gamepad();
    // Gamepad currentGamePad1 = new Gamepad();

    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        starting_time = runtime.time();

        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");
        // rightIntake = hardwareMap.get(CRServo.class,"WheelRight");
        // leftIntake = hardwareMap.get(CRServo.class,"WheelLeft");


        theoreticalAngle = getAngle();

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

        sleeveNum = sleeveDetection();

        // run a path based on sleeve recognition
        if (sleeveNum == 1) {
            oneLeft();
        }
        else if (sleeveNum == 2) {
            twoLeft();
        }
        else if (sleeveNum == 3) {
            threeLeft();
        }
    }

    // 132 cm is 1 foot
    // counter-clockwise is positive
    private void oneLeft() {
        runStraight(11, 90); // 6
        turn(90);
        runStraight(60, 90);
        turn(-90);
        runStraight(90, 90); // 110
    }

    private void twoLeft() {
        runStraight(116, 90);
        // turn(90);
    }

    private void threeLeft() {
        runStraight(11, 90);
        turn(-90);
        runStraight(48, 90);
        turn(88.5);
        runStraight(90, 90);
    }

    private void oneRight() {
        runStraight(11, 90); // 6
        turn(90);
        runStraight(50, 90);
        turn(-90);
        runStraight(90, 90); // 110
    }

    private void twoRight() {
        turn(-3);
        runStraight(116, 90);
        // turn(90);
    }

    private void threeRight() {
        runStraight(11, 90);
        turn(-90);
        runStraight(54, 90);
        turn(89);
        runStraight(90, 90);
    }

    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * 23.7671 * drive);
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

        if (FRPower<0) {
            FRM.setTargetPosition(-ticks);
        }
        else {
            FRM.setTargetPosition(ticks);
        }

        if (FLPower<0) {
            FLM.setTargetPosition(-ticks);
        }
        else {
            FLM.setTargetPosition(ticks);
        }

        if (BRPower<0) {
            BRM.setTargetPosition(-ticks);
        }
        else {
            BRM.setTargetPosition(ticks);
        }

        if (BLPower<0) {
            BLM.setTargetPosition(-ticks);
        }
        else {
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

    public void turn(double degrees){
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentAngle = getAngle();
        double targetAngle = theoreticalAngle + degrees;
        double motorPower = turningPower;

        while (currentAngle<(targetAngle-errorMargin) || currentAngle>(targetAngle+errorMargin))
        {
            if (Math.abs(targetAngle - currentAngle) < 5) {
                motorPower = 0.2;
            }
            else {
                motorPower = turningPower;
            }

            if (currentAngle<targetAngle-errorMargin) {
                FLM.setPower(motorPower * -drive);
                BLM.setPower(motorPower * -drive);
                FRM.setPower(-motorPower * -drive);
                BRM.setPower(-motorPower * -drive);
            }
            if (currentAngle>targetAngle+errorMargin) {
                FLM.setPower(-motorPower * -drive);
                BLM.setPower(-motorPower * -drive);
                FRM.setPower(motorPower * -drive);
                BRM.setPower(motorPower * -drive);
            }

            telemetry.addData("TARGET ANGLE", targetAngle);
            telemetry.addData("CURRENT ANGLE", getAngle());
            telemetry.update();

            currentAngle = getAngle();
        }

        theoreticalAngle += degrees;

        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);
    }

    public int sleeveDetection() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }
        while ((runtime.time() - starting_time) < 12) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        if (recognition.getLabel() == "Triangle") {
                            return 1;
                        }
                        else if (recognition.getLabel() == "Square") {
                            return 3;
                        }
                        else {
                            return 2;
                        }
                    }
                    telemetry.update();
                }
            }
        }

        return 2;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}