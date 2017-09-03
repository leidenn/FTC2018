/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

@TeleOp(name = "TeleOpArms", group = "Test Programs")
public class TeleOpArms extends OpMode {

    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */

    /* TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.20;
	final static double ARM_MAX_RANGE  = 0.70;
	final static double CLAW_MIN_RANGE  = 0.20;
	final static double CLAW_MAX_RANGE  = 0.7;
*/

	//position of the arm servo.
	// double rightArmPosition;
    // double leftArmPosition;

	// amount to change the arm servo position.
	// double rightArmDelta = 0.1;
   //  double leftArmDelta = 0.1;

	// position of the claw servo
	//double LeftClawPosition;

	// amount to change the claw servo position by
	//double LeftClawDelta = 0.1;

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor motorSlide;
 //   DcMotor motorSlideLeft;
   // DcMotor motorDebrisArm;
    Servo servoArmRight;
    Servo servoArmLeft;
  //  Servo servoArmRight2;
  //  Servo servoArmLeft2;
  //  Servo servoButtonRight;
    //Servo servoButtonLeft;

    //Servo arm;

    /**
     * Constructor
     */
    public TeleOpArms() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorSlide = hardwareMap.dcMotor.get("MotorSlide");
        //motorSlideLeft = hardwareMap.dcMotor.get("SlideLeft");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
      //  motorDebrisArm = hardwareMap.dcMotor.get("DebrisArm");
        servoArmRight = hardwareMap.servo.get("ServoArmRight");
        servoArmLeft = hardwareMap.servo.get("ServoArmLeft");
     //   servoArmRight2 = hardwareMap.servo.get("ServoArmRight2");
      //  servoArmLeft2 = hardwareMap.servo.get("ServoArmLeft2");
      //  servoButtonRight = hardwareMap.servo.get("ServoButtonRight");
        //servoButtonLeft = hardwareMap.servo.get("ServoButtonLeft");

        // assign the starting position of the wrist and claw
     //   rightArmPosition = 0.2;
       // leftArmPosition = 0.2;
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        //these are for the drive motors
        float throttleL = -gamepad1.left_stick_y;
       // float direction = gamepad1.left_stick_x;
        float throttleR = -gamepad1.right_stick_y;
       // float directionR = gamepad1.right_stick_x;
      //  float right = throttleR - directionR;
       // float left = throttle + direction;
        float throttleSlide = -gamepad2.right_stick_y;


        // clip the right/left values so that the values never exceed +/- 1
      //  throttleR = Range.clip(throttleR, -1, 1);
      //  throttleL = Range.clip(throttleL, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        throttleR = (float)scaleInput(throttleR);
        throttleL = (float)scaleInput(throttleL);
        throttleSlide = (float)scaleInput(throttleSlide);

        // write the values to the motors
        motorRightFront.setPower(-throttleR);
        motorLeftFront.setPower(-throttleL);
        motorRightBack.setPower(-throttleR);
        motorLeftBack.setPower(-throttleL);
        motorSlide.setPower(-throttleSlide);

        //drive motors ends here
        // arm motors start here
      //  float throttleArmL = -gamepad2.left_stick_y;
       // float directionArmL = gamepad2.left_stick_x;
      //  float throttleArmR = -gamepad2.right_stick_y;
       // float directionArmR = gamepad2.right_stick_x;
        //float rightArms = throttleArmR - directionArmR;
      //  float leftArms = throttleArmL + directionArmL;
      //  motorSlideRight.setPower(-throttleArmR);
        //motorSlideLeft.setPower(-throttleArmL);
       // throttleArmR = Range.clip(throttleArmR, -1, 1);
      //  throttleArmL = Range.clip(throttleArmL, -1, 1);

      //  throttleArmR = (float)scaleInput(throttleArmR);
      //  throttleArmL = (float)scaleInput(throttleArmL);
     //  motorArmRight.setPower(throttleArmR);
     //  motorArmLeft.setPower(-throttleArmL);

        // update the position of the arm.h
		if (gamepad1.a) {
            //if the A button is pushed on gamepad1, increment the position of
            //the arm servo.
            //   rightArmPosition=0.7;
            //	rightArmPosition += rightArmDelta;


            //  leftArmPosition=0.7;
            //  leftArmPosition += leftArmDelta;

         //   servoButtonLeft.setPosition(0.4);
           // servoButtonRight.setPosition(0.94);
            servoArmRight.setPosition(0);
            servoArmLeft.setPosition(1);
          //  servoArmRight2.setPosition(0);
          //  servoArmLeft2.setPosition(1);


        }
		if (gamepad1.y) {
			// if the Y button is pushed on gamepad1, decrease the position of
			// the arm servo.
            //RightClawPosition=0.2;
            servoArmRight.setPosition(1.5);
            servoArmLeft.setPosition(-0.5);

          //  servoArmRight2.setPosition(1.5);
            //servoArmLeft2.setPosition(-0.5);
			//RightClawPosition -= RightClawDelta;
         //   servoArm.setPosition(0);
		}

		/*// update the position of the claw
		if (gamepad1.x) {z
			//LeftClawPosition += LeftClawDelta;
           // servoRightClaw.setPosition(0.2);
		}
*/
		if (gamepad1.b) {


            servoArmRight.setPosition(.75);
            servoArmLeft.setPosition(.5);
			//LeftClawPosition -= LeftClawDelta;
            //LeftClawPosition=0.2;
         //   servoArm.setPosition(1);
		}

        // clip the position values so that they never exceed their allowed range.
      //  RightClawPosition = Range.clip(RightClawPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
       // LeftClawPosition = Range.clip(LeftClawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

		// write position values to the wrist and claw servo
	//	servoRightClaw.setPosition(RightClawPosition);
	//	servoLeftClaw.setPosition(LeftClawPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Text", "Running!");
       // telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
       // telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
       telemetry.addData("throttleL tgt pwr",  "throttleL  pwr: " + String.format("%.2f", throttleL));
        telemetry.addData("throttleR tgt pwr", "throttleR pwr: " + String.format("%.2f", throttleR));
       // telemetry.addData("throttleArmL tgt pwr",  "throttleArmL  pwr: " + String.format("%.2f", throttleArmL));
        //telemetry.addData("throttleArmR tgt pwr", "throttleArmR pwr: " + String.format("%.2f", throttleArmR));
      // telemetry.addData("servoArm", servoArm.getPower());
       // telemetry.addData("servoArm", "servoArm: " + String.format("%.2f", servoArm));
       // telemetry.addData("servoLeftClaw", "servoLeftClaw: " + String.format("%.2f", servoLeftClaw));
// telemetry.addData("Grappling Hook Winch", grapplingWinch.getPower());
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
