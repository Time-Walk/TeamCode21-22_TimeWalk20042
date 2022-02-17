package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="СН.ДОСТАВКА,ЗАГРЁБ,ДОСТАВКА,СКЛАД", group="СИНИЙ")
public class BlueGruzTakeGruzToSklad extends LinearOpMode { //YOU SHOULD CHANGE HERE TO YOUR FILE NAME
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.goForward(500, -1);
        R.drop();
        R.rotate(-40, 0.8);
        R.goForward(1000, 1);
        R.vlRot();
        R.goForward(1000, -1);
        R.drop();
        R.goForward(1000, 1);
        R.vlRot();


    }

}
