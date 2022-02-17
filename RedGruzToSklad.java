package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="КР.ДОСТАВКА,СКЛАД", group="КРАСНЫЙ")
public class RedGruzToSklad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.goForward(500, -1);
        R.drop();
        R.rotate(40, 0.8);
        R.goForward(2000, 1);
        R.vlRot();

    }

}
