package org.firstinspires.ftc.teamcode;

import androidx.appcompat.app.ActionBar;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot2021 extends Robot {
    DcMotor RF, LF, RB, LB, UP, VL, LT; //Создание перременных: моторы
    Servo boxServo; //Серво
    BNO055IMU imu; //Акселерометр

    Orientation angles;
    Acceleration gravity;

    double vlpw=0; //Мощность для вала

    double obg=0;

    @Override
    void init() { //Инициализация:
        UP = hwmp.get(DcMotor.class, "UP"); //Моторов
        LF = hwmp.get(DcMotor.class, "LF");
        LB = hwmp.get(DcMotor.class, "LB");
        RB = hwmp.get(DcMotor.class, "RB");
        RF = hwmp.get(DcMotor.class, "RF");
        VL = hwmp.get(DcMotor.class, "VL");
        LT = hwmp.get(DcMotor.class, "LT");
        boxServo = hwmp.get(Servo.class, "BS"); //Серво
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Режим остоновки: торможение
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        UP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометра
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwmp.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();

    }

    @Override
    void initFields(Telemetry telemetry, LinearOpMode L, HardwareMap hwmp) { //Инициализация
        this.telemetry = telemetry;
        this.L = L;
        this.hwmp = hwmp;

    }

    void setMtPower(double rb,double rf,double lf,double lb){ //Устоновить мощность на моторы
        RF.setPower(rf);
        LF.setPower(lf);
        RB.setPower(rb);
        LB.setPower(lb);
    }

    void wheelbase (){ //Функция мощностей считывания с джостика
        double l = gamepad1.left_stick_x - gamepad1.left_stick_y;
        double r = gamepad1.left_stick_x + gamepad1.left_stick_y;
        setMtPower(r, r, l, l);

    }

    void goForward (long x, double pw){ //Функция автонома: ехать вперед (можно и назад)
        setMtPower(-pw, -pw, pw, pw);
        L.sleep(x);
        setMtPower(0, 0, 0, 0);
    }

    double getAngle() { //Функция получения данных с акселерометра
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }

    void rotate (double degrees) { //Функция автонома: поворот
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && L.opModeIsActive()) {
                if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
                if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

                double Er = degrees - (getAngle());

                double kp = 0.9;
                double P = kp * Er / Er0 * pw;

                double kd = -0.05;
                double ErD = Er - ErLast;
                double D = kd * ErD * (1/Er);

                if (Math.signum(D) > Math.signum(P)) {  D=P; }

                double kr = -0.05;
                double Rele = kr * Math.signum(Er);

                ErLast = Er;


                double pwf = P+D+Rele; //Регулятор


                setMtPower(pwf, pwf, pwf, pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

            }
        setMtPower(0, 0, 0, 0);
        delay(500);
    }

    void duckVoid(double pw) { //Функция автонома: скидывание уточки
        UP.setPower(pw);
        delay(3500);
        UP.setPower(0);
    }


    void DEBUG() { //Функция для дебагинга
        /*if ( gamepad1.x ) {
        }
        if ( gamepad1.b ) {
        }*/
    }

    void smartRotate() { //Помощь для драйверов
        if ( gamepad1.x ) {
            rotate(-90);
        }
        if ( gamepad1.b ) {
            rotate(90);
        }
    }

    void valController() { //Поворот вала
        vlpw=0;
        if ( gamepad2.left_bumper ) {
            vlpw = 0.7;
        }
        if ( gamepad2.right_bumper ) {
            vlpw = -0.7;
        }
        VL.setPower(vlpw);
    }


    void servoController() { //Открытие коробки
        if ( gamepad2.dpad_up ) {
            boxServo.setPosition(0.65);
            delay(100);
            boxServo.setPosition(1);
        }
        if ( gamepad2.dpad_down ) {
            boxServo.setPosition(0.4);
        }
    }

    Thread liftControllerT = new Thread() { //Поток для лифта
        @Override
        public void run() {
            double Er = 0;
            double ErLast = 0;
            double tt=0;
            while (L.opModeIsActive() && !L.isStopRequested()) {
                LT.setPower(gamepad2.left_stick_y/-2.5); //Управление лифтом стиком
                if (gamepad2.y) { //Поднять до конца
                    LT.setPower(0.6);  //начальное ускорение
                    delay(450);
                    LT.setPower(0.35);    //спокойная скорость
                    delay(400);
                    LT.setPower(0);      //стоп
                }
                if (gamepad2.a) { //Опустить
                    LT.setPower(-0.3);
                    delay(900);
                    LT.setPower(0);
                }
                if (gamepad2.left_stick_y == 0 && !gamepad2.y && !gamepad2.a) {
                    Er = LT.getCurrentPosition();
                    double kd = - 1;
                    double pwf = kd * (Er - ErLast);
                    tt += 1;
                    LT.setPower(pwf);
                    telemetry.addData("getCurr", UP.getCurrentPosition());
                    telemetry.addData("pwf", pwf);
                    telemetry.addData("Er-ErLast", Er-ErLast);
                    telemetry.addData("Entry", tt);
                    telemetry.update();
                    ErLast = Er;
                }
            }
        }
    };

    void go(double cm) { //
        double pw = 1;
        double cc = (400 * cm) / 32.97;
        double Er0 = cc;
        double errorFix=0;
        double ErLast = 0;
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( cc - LB.getCurrentPosition() > 10 && L.opModeIsActive()) {

            double Er = cc - LB.getCurrentPosition();

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.1;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = pw * (P+D+Rele); //Регулятор

            setMtPower(-pwf, -pwf, pwf, pwf);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.update();*/

        }
        setMtPower(0, 0, 0, 0);
        delay(500);
    }

    void back(double cm) { //
        double pw = 1;
        double cc = -(400 * cm) / 32.97;
        double Er0 = -cc;
        double errorFix=0;
        double ErLast = 0;
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( Math.abs(cc - LB.getCurrentPosition()) > 10  && L.opModeIsActive()) {

            double Er = cc - LB.getCurrentPosition();

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.1;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = pw * (P+D+Rele); //Регулятор

            setMtPower(-pwf, -pwf, pwf, pwf);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.addData("m", Math.abs(cc-LB.getCurrentPosition()));
            telemetry.update();*/

        }
        setMtPower(0, 0, 0, 0);
        delay(500);
    }


    void drop() { //Функция автонома: скидывание
        boxServo.setPosition(0.2);
        //подъём коробки
        LT.setPower(0.6);  //начальное ускорение
        VL.setPower(-0.7);
        delay(500);
        LT.setPower(0.35);    //спокойная скорость
        delay(400);
        LT.setPower(0);      //стоп
        VL.setPower(0);
        delay(500);
        //серво-открыть
        boxServo.setPosition(0.65);
        delay(100);
        boxServo.setPosition(1);
        delay(1000);
        //серво-закрыть
        boxServo.setPosition(0.4);
        delay(700);
        go(20);        //опускание коробки
        LT.setPower(-0.5);
        VL.setPower(0.5);
        delay(900);
        VL.setPower(0);
        LT.setPower(0);
    }

    void vlRot() { //Функция автонома: поворот вала
        VL.setPower(-0.7);
        delay(2000);
        VL.setPower(0);
    }

    void rotateForTime(long x, double pw) {
        setMtPower(pw, pw, pw, pw);
        delay(x);
        setMtPower(0, 0, 0, 0);
    }

}


