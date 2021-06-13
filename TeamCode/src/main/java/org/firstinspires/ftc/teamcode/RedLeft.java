//Run code for the red left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "RedLeft", group = "Taus")
public class RedLeft extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 14);//16
                shoot(0, (2195*28)/60.0);
                forward(.5, 0, 14);//12
                strafeRight(.5, 1, 0);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 14);//16
                shoot(0, (2195*28)/60.0);
                toAngle(2, 1);
                forward(.5, 1, 17);//15
                toAngle(-150, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(-180, 1);
                forward(.5, 2, 15);
                toAngle(270, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(360, 1);
                forward(.5, 0, 20);
                shoot(2, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 2, 14);//16
                shoot(0, (2195*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 10);//8
                strafeRight(.5, 1, 0);
                toAngle(165, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                toAngle(180, 1);
                forward(.5, 3, 0);//2
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 16);//10
                forward(.5, 0, 2);
                //backward(.5, 0, 4);
                //forward(.5, 0, 2);
                backward(.5, 0, 4);
                forward(.5, 0, 6);
                shoot(-6, (2300*28)/60.0);
                setIntakePower(1);
                toAngle(-90, 1);
                backward(.5, 0, 11);
                forward(.5, 0, 17);
                //toAngle(0, 1);
                shoot(-6, (2195*28)/60.0);//not really
//                forward(.5, 0, 20);//23
//                shoot(-5, (2110*28)/60.0);
//                toAngle(0, 1);
//                forward(.5, 0, 12);
                break;
        }

    }
}
