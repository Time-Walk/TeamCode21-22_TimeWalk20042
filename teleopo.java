package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        telemetry.clearAll();
        telemetry.update();
        while (!isStopRequested()){
            R.UP.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            R.LT.setPower(gamepad2.left_stick_y/3);
            R.wheelbase();
            R.servoController();
            R.smartRotate();
            R.valController();
            //R.DEBUG();
        }

    }
}
