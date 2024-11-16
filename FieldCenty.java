package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class FieldCenty extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder

    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.

    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    boolean slowMode = false;









    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("bR");

        CRServo spinnyL = hardwareMap.crservo.get("spinnyL");
        CRServo spinnyR = hardwareMap.crservo.get("spinnyR");


        DcMotor uppies = hardwareMap.dcMotor.get("uppies");

        Servo inL = hardwareMap.servo.get("inL");
        Servo inR = hardwareMap.servo.get("inR");


        Servo linkL = hardwareMap.servo.get("linkL");
        Servo linkR = hardwareMap.servo.get("linkR");

        Servo uppiesJointL = hardwareMap.servo.get("uppiesJointL");
        Servo uppiesJointR = hardwareMap.servo.get("uppiesJointR");


        Servo bucket = hardwareMap.servo.get("bucket");






        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bucket.setDirection(Servo.Direction.REVERSE);

        inL.setDirection(Servo.Direction.REVERSE);
        linkL.setDirection(Servo.Direction.REVERSE);


        spinnyR.setDirection(CRServo.Direction.REVERSE);










        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {



            double triggerVal = gamepad1.left_trigger;
            double powerFactor = slowMode ? 0.2 : 0.5;


            if (triggerVal > 0.1) {
                // Map trigger value from [0.1, 1] to [0.5, 1] for finer control
                powerFactor = 0.2 + 0.2 * (1 - triggerVal);
            }





            if (gamepad2.left_bumper){
                linkL.setPosition(1);
                linkR.setPosition(1);
            } else {
                linkL.setPosition(0);
                linkR.setPosition(0);
            }


           



            if (gamepad2.y){
                inL.setPosition(0);
                inR.setPosition(0);
            } else {
                inL.setPosition(1);
                inR.setPosition(1);
            }




            if (gamepad2.right_bumper){
                spinnyL.setPower(1);
                spinnyR.setPower(1);
            } else {
                spinnyL.setPower(0);
                spinnyR.setPower(0);
            }

            if (gamepad2.a){
                spinnyL.setPower(-1);
                spinnyR.setPower(-1);
            } else {
                spinnyL.setPower(0);
                spinnyR.setPower(0);
            }












            if (Math.abs(gamepad2.right_stick_y) > .1){
                uppies.setPower(gamepad2.right_stick_y * 0.7);
            } else{
                uppies.setPower(0);
            }






















            double y = -gamepad1.left_stick_y ; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ;
            double rx = -gamepad1.right_stick_x ;
            y *= powerFactor;
            x *= powerFactor;
            rx *= powerFactor;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);





        }
    }
}

