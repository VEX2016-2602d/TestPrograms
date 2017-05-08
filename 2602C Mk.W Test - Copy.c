#pragma config(Sensor, in1,    degreeR,        sensorPotentiometer)
#pragma config(Sensor, in2,    degreeL,        sensorPotentiometer)
#pragma config(Sensor, in3,    backUpBattery,  sensorAnalog)
#pragma config(Sensor, dgtl1,  ,               sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  encoderArm,     sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, encoderR,       sensorQuadEncoder)
#pragma config(Motor,  port2,           _pincherL,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           _pincherR,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           _armL,         tmotorVex393_MC29, openLoop, encoderPort, dgtl7)
#pragma config(Motor,  port5,           _armR,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           _chassisR_M,   tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port7,           _chassisL_M,   tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port8,           _chassisL,     tmotorVex393HighSpeed_MC29, openLoop, driveLeft, encoderPort, dgtl9)
#pragma config(Motor,  port9,           _chassisR,     tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight, encoderPort, dgtl11)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "SmartMotorLib.c"
#include "CoreLib.c"

void _armDrive(int speed)
{
	motor[_armR]=speed;
	motor[_armL]=speed;
}

void _pincherDrive(int speed)
{
	motor[_pincherR]=speed;
	motor[_pincherL]=speed;
}

void smartMotorSetUp()
{
	SmartMotorsInit();
	SmartMotorsAddPowerExtender(_pincherL,_pincherR,_armL,_armR);
	SmartMotorLinkMotors(_chassisL,_chassisL_M);
	SmartMotorLinkMotors(_chassisR,_chassisR_M);
	SmartMotorSetSlewRate(_chassisR_M,255);
	SmartMotorSetSlewRate(_chassisL_M,255);
	SmartMotorSetSlewRate(_chassisL,255);
	SmartMotorSetSlewRate(_chassisR,255);
	SmartMotorCurrentMonitorEnable();
	SmartMotorSetPowerExpanderStatusPort(in3);
	SmartMotorRun();
}

void pincherSetUp()
{
	setPincher(0,port3,in1,2300,3000,3700,true);
	setPincher(1,port2,in2,2250,3000,3700,true);
}
void chassisSetUp()
{
	setChassis(0,port6,port9,dgtl11,0.45,0.00001,1,50);
	setChassis(1,port7,port8,dgtl9,0.4,0.00001,1,50);
}
task main()
{
	smartMotorSetUp();
	pincherSetUp();
	//chassisSetUp();
	//chassisPID(true,false,false,800);
  while(true) // Space between while (true) deleted
  {
    //int _ch2=vexRT[Ch2];
    //int _ch3=vexRT[Ch3];
    //motor[_chassisL]=_ch3;
    //motor[_chassisL_M]=_ch3;
  	/*motor[_chassisL]=vexRT[Ch3];
  	motor[_chassisL_M]=vexRT[Ch3];
  	motor[_chassisR]=vexRT[Ch2];
  	motor[_chassisR_M]=vexRT[Ch2];*/

    SetMotor(_chassisR,vexRT[Ch2]);
    SetMotor(_chassisR_M,vexRT[Ch2]);
    SetMotor(_chassisL,vexRT[Ch3]);
    SetMotor(_chassisL_M,vexRT[Ch3]);


    if(vexRT[Btn6U] ==1)
    {
      _armDrive(125);
    }

    else if(vexRT[Btn6D]==1)
    {
      _armDrive(-120);
    }

    else
    {
      _armDrive(0);
    }

    if(vexRT[Btn5U] ==1)
    {
    	//stopTask(openPincher);
      //startTask(closePincher);
    	pincherDrive(120);
    }

    else if(vexRT[Btn5D]==1)5\
    {
    	//stopTask(closePincher);
     	//startTask(openPincher);
     	pincherDrive(-120);
    }
    else
    {
      _pincherDrive(0);
    }
  }
}
