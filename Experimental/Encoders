package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Fart Method", group="Linear Opmode")
//@Disabled
public class Fart extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorL;
    private DistanceSensor sensorR;
    DcMotor topL;
    DcMotor topR;
    DcMotor backL;
    DcMotor backR;
    DcMotor arm;
    DcMotor rotate;
    DcMotor arm2;
    Servo hand;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        topL = hardwareMap.dcMotor.get("topLeft");
//        topR = hardwareMap.dcMotor.get("topRight");
//        backL = hardwareMap.dcMotor.get("backLeft");
//        backR = hardwareMap.dcMotor.get("backRight");
//        arm = hardwareMap.dcMotor.get("arm");
//        arm2 = hardwareMap.dcMotor.get("arm2");
//        hand = hardwareMap.servo.get("hand");
//        rotate = hardwareMap.dcMotor.get("rotate");
        sensorL = hardwareMap.get(DistanceSensor.class, "sensorL");
        sensorR = hardwareMap.get(DistanceSensor.class, "sensorR");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftN = 0;
        double rightN = 0;
        telemetry.addData("range", String.format("%.01f m", sensorL.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f m", sensorR.getDistance(DistanceUnit.METER)));
        leftN =  leftN + sensorL.getDistance(DistanceUnit.METER);
        rightN = rightN + sensorR.getDistance(DistanceUnit.METER);
        telemetry.update();

        topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topR.setTargetPosition(5000);
//        topL.setTargetPosition(5000);
//        backR.setTargetPosition(5000);
//        backL.setTargetPosition(5000);

        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(6000);

        telemetry.addData("range", String.format("%.01f m",rightN));
        telemetry.addData("range", String.format("%.01f m",leftN));
        telemetry.update();




    }
    public void strafeL(double pow, int time)
    {
        double negative = pow * -1;
        topL.setPower(pow);
        backL.setPower(negative);
        topR.setPower(negative);
        backR.setPower(pow);
        sleep(time);
    }
    public void strafeR(double pow, int time)
    {
        double negative = pow * -1;
        topL.setPower(negative);
        backL.setPower(pow);
        topR.setPower(pow);
        backR.setPower(negative);
        sleep(time);
    }
    public void straitF(double pow, int time)
    {
        double negative = pow * -1;
        topL.setPower(negative);
        topR.setPower(pow);
        backL.setPower(negative);
        backR.setPower(pow);
        sleep(time);
    }
    public void straitB(double pow, int time)
    {
        double negative = pow * -1;
        topL.setPower(pow);
        topR.setPower(negative);
        backL.setPower(pow);
        backR.setPower(negative);
        sleep(time);
    }
    public void armUP(double pow, int time)
    {
        double negative = pow * -1;
        arm.setPower(pow);
        arm2.setPower(negative);
        sleep(time);
    }
    public void armDown(double pow, int time)
    {
        double negative = pow * -1;
        arm.setPower(negative);
        arm2.setPower(pow);
        sleep(time);
    }
    public void turn90R()
    {
        topL.setPower(-.5);
        topR.setPower(-.5);
        backL.setPower(-.5);
        backR.setPower(-.5);
        sleep(600);
    }
    public void turn90L()
    {
        topL.setPower(.5);
        topR.setPower(.5);
        backL.setPower(.5);
        backR.setPower(.5);
        sleep(700);
    }
    public void stopM()
    {
        topL.setPower(0);
        topR.setPower(0);
        backL.setPower(0);
        backR.setPower(0);
    }
    public void armTop()
    {
        //Raises the robots arm to top
        arm.setPower(.3);
        arm2.setPower(-.3);
        sleep(1400);
    }
    public void armMid()
    {
        //Raises the robots arm to middle
        arm.setPower(.3);
        arm2.setPower(-.3);
        sleep(850);
    }
    public void armLow()
    {
        //Raises the robots arm to bottom
        arm.setPower(.3);
        arm2.setPower(-.3);
        sleep(350);
    }
    public void DriveForwardDistance (double power, int distance)
    {
        topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topR.setTargetPosition(5000);
        topL.setTargetPosition(5000);
        backR.setTargetPosition(5000);
        backL.setTargetPosition(5000);

        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
