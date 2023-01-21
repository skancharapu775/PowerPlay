package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="slideCalibration", group="--")
@Disabled
public class slideCalibration extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx slide = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        slide = hardwareMap.get(DcMotorEx.class, "liftMotor");

        //GamePads to save previous state of gamepad for toggling slug mode
        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            // linear slide
            if (currentGamePad1.right_trigger > 0) {
                slide.setPower(-currentGamePad1.right_trigger);
            }
            else if (currentGamePad1.left_trigger > 0) {
                slide.setPower(currentGamePad1.left_trigger);
            }
            else {
                slide.setPower(0);
            }

            telemetry.update();
        }
    }

}