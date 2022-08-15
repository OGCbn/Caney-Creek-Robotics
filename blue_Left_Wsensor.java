package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="Blue Left", group="Linear Opmode")
//@Disabled
public class blue_Left_Wsensor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor topL;
    DcMotor topR;
    DcMotor backL;
    DcMotor backR;
    DcMotor arm;
    DcMotor rotate;
    DcMotor arm2;
    Servo hand;
    private DistanceSensor sensorL;
    private DistanceSensor sensorR;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        topL = hardwareMap.dcMotor.get("topLeft");
        topR = hardwareMap.dcMotor.get("topRight");
        backL = hardwareMap.dcMotor.get("backLeft");
        backR = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        arm2 = hardwareMap.dcMotor.get("arm2");
        hand = hardwareMap.servo.get("hand");
        rotate = hardwareMap.dcMotor.get("rotate");
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
        telemetry.addData("range", String.format("%.01f m",rightN));
        telemetry.addData("range", String.format("%.01f m",leftN));
        telemetry.update();
        //Makes the bot go forward
        straitF(.37,800);
        //Stops the robot
        stopM();
        sleep(100);
        //Raises the robots arm
        if(rightN < 1.5)
        {
            armTop();
        }
        else if(leftN < 1.5)
        {
            armMid();
        }
        else if((rightN > 1.5) && (leftN > 1.5))
        {
            armLow();
        }
        //Makes the bot go forward
        topL.setPower(-.38);
        backL.setPower(-.38);
        topR.setPower(.38);
        backR.setPower(.38);
        arm.setPower(0);
        arm2.setPower(0);
        sleep(1800);
        stopM();
        sleep(50);
        turn90R();
        straitF(.2,400);
        stopM();
        sleep(50);
        //Stops the bot and opens the hand
        hand.setPosition(1);
        topL.setPower(0);
        topR.setPower(0);
        backR.setPower(0);
        backL.setPower(0);
        sleep(2000);
        //Reverse
        topL.setPower(.38);
        backL.setPower(.38);
        topR.setPower(-.38);
        backR.setPower(-.38);
        sleep(700);
        //Stop robot
        topL.setPower(0);
        backL.setPower(0);
        topR.setPower(0);
        backR.setPower(0);
        sleep(100);
        //Stops the robots arm
        arm.setPower(0);
        arm2.setPower(0);
        sleep(100);
        //Makes the bot strafe left
        strafeR(.27,7500);
        strafeL(.27,400);
        turn90L();
        stopM();
        armDown(.5,500);
        stopM();
        armLow();
        //Makes the bot go forward
        sleep(10);
        stopM();
        straitB(.27,1000);
        stopM();
        sleep(150);
        strafeL(.1,300);
        strafeL(.4,3800);
        sleep(50);
        armLow();
        stopM();
        sleep(150);
        stopM();
        straitF(.5,1000);
        stopM();
        sleep(50);
        armDown(.7,200);
        turn90L();





        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;
//
//            // Choose to drive using either Tank Mode, or POV Mode
//            // Comment out the method that's not used.  The default below is POV.
//
//            // POV Mode uses left stick to go forward, and right stick to turn.
//            // - This uses basic math to combine motions and is easier to drive straight.
////            double drive = -gamepad1.left_stick_y;
////            double turn  =  gamepad1.right_stick_x;
////            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
////            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
////
////            // Tank Mode uses one stick to control each wheel.
////            // - This requires no math, but it is hard to drive forward slowly and keep straight.
////            // leftPower  = -gamepad1.left_stick_y ;
////            // rightPower = -gamepad1.right_stick_y ;
////
////            // Send calculated power to wheels
////            leftDrive.setPower(leftPower);
////            rightDrive.setPower(rightPower);
//
//            topL.setPower(1);
//            topR.setPower(1);
//            backL.setPower(-1);
//            backR.setPower(-1);
//            sleep(1000);
//
//            sleep(1000);
//
//            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.update();
//        }
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
        sleep(600);
    }
    public void stopM()
    {
        topL.setPower(0);
        topR.setPower(0);
        backL.setPower(0);
        backR.setPower(0);
        arm.setPower(0);
        arm2.setPower(0);
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
}
