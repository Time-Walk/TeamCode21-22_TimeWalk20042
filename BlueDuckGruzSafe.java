package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="СН.УТКА,ГРУЗ,СЕЙФ", group="СИНИЙ")
@Disabled
public class BlueDuckGruzSafe extends LinearOpMode { //YOU SHOULD CHANGE HERE TO YOUR FILE NAME
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.goForward(130, -0.25);
        R.duckVoid(1);
        //R.rotate(-25, 0.5);
        R.goForward(400, -1);
        R.delay(1000);
        //R.rotate(-50, 0.5);
        R.goForward(500, -0.8);
        R.drop();
        R.rotateForTime(300, -0.5);
        R.goForward(550, 1);


    }

}
