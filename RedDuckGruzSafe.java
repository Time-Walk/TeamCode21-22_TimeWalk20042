package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="КР.УТКА,ГРУЗ,СЕЙФ", group="КРАСНЫЙ")
@Disabled
public class RedDuckGruzSafe extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go
        R.goForward(130, 0.25);
        R.duckVoid(-1);
        //R.rotate(20, 0.8);
        R.goForward(500, 1);
        R.delay(1000);
        //R.rotate(-83, 0.3);
        R.goForward(500, -1);
        R.drop();
        R.goForward(575, 1);


    }

}
