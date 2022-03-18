package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="ПО ПРИКАЗУ ГЕНЕРАЛА ГАВСА, БОЛЬШИЕ ГОРОДА", group="")
public class OpenBox extends LinearOpMode { //YOU SHOULD CHANGE HERE TO YOUR FILE NAME
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        R.boxServo.setPosition(1);
        R.delay(5000);
        R.boxServo.setPosition(0.4);
        R.delay(1000);


    }

}
