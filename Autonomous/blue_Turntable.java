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

@Autonomous(name="Blue Turntable", group="Linear Opmode")
//@Disabled
public class blue_Turntable extends LinearOpMode {

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
        //Makes the bot strafe left toward the shipping hub
        //topL.setPower(.37);topR.setPower(-.38);backR.setPower(.38);backL.setPower(-.38);sleep(3000);
        straitF(.37, 800);
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
        sleep(2000);
        stopM();
        sleep(50);
        turn90L();
        straitF(.27, 850);
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
        sleep(1000);
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
        armDown(.5, 200);
        straitB(.3, 3200);
        stopM();
        strafeL(.3, 4200);
        stopM();
        straitB(.4,800);
        stopM();
//        turn90R();
//        stopM();
//        strafeL(.4,500);
//        stopM();
//        straitB(.2,800);
//        stopM();
//        turn90L();
        strafeL(.3,1400);
        rotate.setPower(.7);
        stopM();
        sleep(3500);
        //stop rotation
        rotate.setPower(0);
        sleep(50);
        strafeR(.8, 700);

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
        sleep(800);
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
}
