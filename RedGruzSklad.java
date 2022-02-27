package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Красный.Груз,Склад", group="К")
public class RedGruzSklad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2021 R = new Robot2021();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.back(50);
        R.drop();
        R.back(10);
        R.rotate(80);
        R.goForward(1200, -0.9);
	


    }

}
