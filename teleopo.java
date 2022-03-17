package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp")
public class teleopo extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.gamepad_init(gamepad1,gamepad2);
        waitForStart();
        R.liftControllerT.start();
        while (!isStopRequested()){
            R.UP.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            R.wheelbase();
            R.servoController();
            R.smartRotate();
            R.valController();
            R.DEBUG();
        }

    }
}
