#ifndef _CORELIB_;
#define _CORELIB_;

typedef struct
{
  tMotor motorPort;
  tSensors sensorPort;
  float kP;
  float kI;
  float kD;
  int integralLimit;
}chassisData;

typedef struct
{
  tMotor motorPort;
  tSensors sensorPort;
  int open;
  int mid;
  int close;
  bool ifHold;
}clawData;

typedef struct
{
  tMotor motorPortR;
  tMotor motorPortL;
  tMotor motorPortR2;
  tMotor motorPortL2;
  tSensors sensorPort;
  int down;
  int full;
  int holdNull;
  int holdStar;
  int hold3Stars;
  int holdCube;
}armData;

typedef struct
{
  tSensors port;
  int scale;
  int angle;
}gyroData;

static armData arm;
static clawData pincherR;
static clawData pincherL;
static chassisData chassisR;
static chassisData chassisL;
static gyroData gyro1;

void setArm(tMotor motorR, tMotor motorL, tSensors port,int down, int full,int none,int star,int stars,int cube)
{
  arm.motorPortR = motorR;
  arm.motorPortL = motorL;
  arm.sensorPort=port;
  arm.down=down;
  arm.full=full;
  arm.holdNull=none;
  arm.holdStar=star;
  arm.hold3Stars=stars;
  arm.holdCube=cube;
}

void setArm(tMotor motorR,tMotor motorR2,tMotor motorL,tMotor motorL2,tSensors port,int down, int full,int none,int star,int stars,int cube)
{
  arm.motorPortR = motorR;
  arm.motorPortR2 = motorR2;
  arm.motorPortL = motorL;
  arm.motorPortL2 = motorL2;
  arm.sensorPort=port;
  arm.down=down;
  arm.full=full;
  arm.holdNull=none;
  arm.holdStar=star;
  arm.hold3Stars=stars;
  arm.holdCube=cube;
}

void setChassis(char side,tMotor motorport, tSensors sensorport, float kp, float ki, float kd, float integrallimit)
{
  if(side==0)
  {
    chassisR.motorPort=motorport;
    chassisR.sensorPort=sensorport;
    chassisR.kP=kp;
    chassisR.kI=ki;
    chassisR.kD=kd;
    chassisR.integralLimit=integrallimit;
  }
  else if(side==1)
  {
    chassisL.motorPort=motorport;
    chassisL.sensorPort=sensorport;
    chassisL.kP=kp;
    chassisL.kI=ki;
    chassisL.kD=kd;
    chassisL.integralLimit=integrallimit;
  }
  else {}
}

void setPincher(char side,tMotor motorport, tSensors sensorport,int openMV,int midMV,int closeMV,bool ifHold)
{
  if(side==0)
  {
    pincherR.motorPort=motorport;
    pincherR.sensorPort=sensorport;
    pincherR.open=openMV;
    pincherR.mid=midMV;
    pincherR.close=closeMV;
    pincherR.ifHold=ifHold;
  }
  else if(side==1)
  {
    pincherL.motorPort=motorport;
    pincherL.sensorPort=sensorport;
    pincherL.open=openMV;
    pincherL.mid=midMV;
    pincherL.close=closeMV;
    pincherL.ifHold=ifHold;
  }
  else {}
}

void setGyro(tSensors port,int scale)
{
  gyro1.port=port;
  gyro1.scale=scale;
}

void armDrive(int armspeed)
{
	motor[arm.motorPortR] = armspeed;
	motor[arm.motorPortL] = armspeed;
}

void pincherDrive(int pincherspeed)
{
	motor[pincherR.motorPort] = pincherspeed;
	motor[pincherL.motorPort] = pincherspeed;
}

void resetChassisEncoders()
{
	SensorValue[chassisR.sensorPort]=0;
	SensorValue[chassisL.sensorPort]=0;
}

void resetArmEncoder()
{
	SensorValue[arm.sensorPort]=0;
}

int pCalc(int target, tSensors sensorName,float pValue)//always return pasitive value
{
	int error = abs(target - SensorValue[sensorName]);
	int index = (int)error * pValue;
	if(index > 125)//prevent overshot the motor
	{
		index = 125;
	}
	else
	{
		index = index;
	}
	return index;
}

void gyroSetup()
{
  SensorType[gyro1.port] = sensorNone;
  wait1Msec(1000);
  SensorType[gyro1.port] = sensorGyro;
  wait1Msec(2000);
  SensorScale[gyro1.port] = gyro1.scale;
}

task gyroFilter()
{
  int     gyro_Read;
  int     gyro_Error = 0;
  int     lastDriftGyro =0;

  int     angle;
  long    nSysTimeOffset;

  nSysTimeOffset = nSysTime;

  while(true)
  {
  	gyro_Read=SensorValue[gyro1.port];
    //if the angle speed smaller than 20/s than consider as a drift
    if( (nSysTime - nSysTimeOffset) > 250 )
      {
        if( abs( gyro_Read - lastDriftGyro ) < 3
        	)
        {
          gyro_Error += (lastDriftGyro - gyro_Read);
        }
        lastDriftGyro = gyro_Read;
        nSysTimeOffset = nSysTime;
      }
      angle = (gyro_Read + gyro_Error)/10;

      //fit the runover
      if(angle< -360)
      {
        angle += 360;
      }
      else if(angle >360)
      {
        angle -= 360;
      }
      else
      {
        angle = angle;
  		}
      gyro1.angle = angle; //store the result to global variable;
      wait1Msec(15);
  }
}

void gyroTurn(int nDegree,int timeLimit)
{
	int error;
  float index;
	int preError = gyro1.angle - nDegree;
	int derivative;
  clearTimer(T2);
  while(time1[T2]<timeLimit)
  {
    error = gyro1.angle - nDegree;
    derivative = error-preError;
    preError = error;
    index = 2*error+5.8*derivative;

  	if(index > 125)//prevent overshot the motor
		{
		index = 125;
		}
		else if(index<-125)
		{
			index = -125;
		}
		else
		{
		index = index;
		}
    motor[chassisL.motorPort] = index;
    motor[chassisR.motorPort] = -index;
    wait1Msec(25);
  }
  motor[chassisL.motorPort] = 0;
  motor[chassisR.motorPort] = 0;
}

void gyroAdjustment(int nDegree)//counterclockwise is postive
{
  int error;
  float index;
  int intergrate=0;
  clearTimer(T1);
  while(time1[T1]<500)
  {
    error = abs(gyro1.angle - nDegree);
    intergrate +=error;
    index = 16*error + 0.1*intergrate;
    if(index>30)
    	index = 30;
   	else
   		index = index;
   if(gyro1.angle>nDegree)
   {
    motor[chassisL.motorPort] = index;
    motor[chassisR.motorPort] = -index;
 		}
 		else if(gyro1.angle<nDegree)
 		{
 		motor[chassisL.motorPort] = -index;
    motor[chassisR.motorPort] = index;
 		}
 		else
 		{
 		motor[chassisL.motorPort] = 0;
    motor[chassisR.motorPort] = 0;
  	}
    wait1Msec(25);
  }
  motor[chassisL.motorPort] = 0;
  motor[chassisR.motorPort] = 0;
}

void chassisPID(bool forward,bool ifLift,bool ifHoldPincher,int target)
{
	float indexR=0.0;
	float indexL=0.0;
	int integralLimit;

	int errorL;
	int preErrorL = target;
	int integralL =0;
	int derivativeL;

	int errorR;
	int preErrorR = target;
	int integralR =0;
	int derivativeR;

	char lowSpeedCountL=0;
	char lowSpeedCountR=0;

	resetChassisEncoders();

	while(((abs(SensorValue[chassisR.sensorPort])) < target) || ((abs(SensorValue[chassisL.sensorPort]))< target))
	{
		errorL = target - abs(SensorValue[chassisL.sensorPort]);
		errorR = target - abs(SensorValue[chassisR.sensorPort]);

		integralL += errorL;
		integralR += errorR;

		if(abs(errorL) < integralLimit)
		{
			integralL = 0;
		}

		if(abs(errorR) < integralLimit)
		{
			integralR = 0;
		}

		derivativeL = errorL - preErrorL;
		derivativeR = errorR - preErrorR;

		preErrorL = errorL;
		preErrorR = errorR;


		indexL = chassisL.kP*errorL + chassisL.kI*integralL + chassisL.kD*derivativeL;
		indexR = chassisR.kP*errorR + chassisR.kP*integralR + chassisR.kD*derivativeR;

		if(errorR < 150 && abs(derivativeR)< 2)
			lowSpeedCountR++;

		if(errorL < 150 && abs(derivativeL)< 2)
			lowSpeedCountL++;

		if(lowSpeedCountL>5 || lowSpeedCountR>5)
			break;

		if(indexR > 125)//prevent overshot the motor
		{
			indexR = 125;
		}
		else
		{
			indexR = indexR;
		}

		if(indexL > 125)
		{
			indexL = 125;
		}
		else
		{
			indexL = indexL;
		}

		if(forward)
		{
      motor[chassisR.motorPort]=indexR;
			motor[chassisL.motorPort]=indexL;
		}
		else
		{
			motor[chassisR.motorPort]=-indexR;
			motor[chassisL.motorPort]=-indexL;
		}

		if(ifLift && errorR <180)
		{
			armDrive(125);
		}

		if(ifHoldPincher && errorR>100)
		{
			pincherDrive(20);
		}
		wait1Msec(25);
	}
	motor[chassisR.motorPort]=0;
	motor[chassisL.motorPort]=0;
}

void autoOpenPincher()
{
	while(SensorValue[pincherL.sensorPort] > pincherL.open  || SensorValue[pincherR.sensorPort] > pincherR.open)
	{
		int openIndexL = pCalc(pincherL.open, pincherL.sensorPort, 0.9);
		int openIndexR = pCalc(pincherR.open, pincherR.sensorPort,0.9);

		if(openIndexL < 20 || openIndexR <20)
		{
			break;
		}

		if(SensorValue[pincherL.sensorPort] > pincherL.open)
		{
			motor[pincherL.motorPort] = -openIndexL;
		}

		if(SensorValue[pincherR.sensorPort] > pincherR.open)
		{
			motor[pincherR] = -openIndexR;
		}
	}
	pincherDrive(0);
}

void autoClosePincher()
{
	int preReadL = pincherL.open-50;//ensure that we have a big speed at start so it won't trigger then break
	int preReadR = pincherR.open-50;
	int speedL;
	int speedR;
	int currentReadL;
	int currentReadR;
	char achievedCountL =0;
	char achievedCountR =0;
	pincherDrive(125);
	wait1Msec(200);
	clearTimer(T4);
	while(time1[T4]<650 &&(SensorValue[pincherL.sensorPort] < pincherL.close || SensorValue[pincherR.sensorPort] < pincherR.close))
	{
		currentReadL = SensorValue[pincherL.sensorPort];
		currentReadR = SensorValue[pincherR.sensorPort];

		speedL = abs(currentReadL - preReadL);
		speedR = abs(currentReadR - preReadR);

		if(SensorValue[pincherL.sensorPort] < pincherL.close)
		{
			if(speedL>5)
			{
				motor[pincherL.motorPort] =125;
			}
			else
			{
				achievedCountL++;
				motor[pincherL.motorPort] = 0;
			}
		}
		else
		{
			motor[pincherL.motorPort] = 0;
		}

		if(SensorValue[pincherR.sensorPort] < pincherR.close)
		{
			if(speedR>5)
			{
				motor[pincherR.motorPort] =125;
			}
			else
			{
				achievedCountR++;
				motor[pincherR.motorPort] = 0;
			}
		}
		else
		{
			motor[pincherR.motorPort] = 0;
		}

		if(achievedCountL >2 && achievedCountR >2)
		{
			break;
		}
		preReadL = currentReadL;
		preReadR = currentReadR;
		wait1Msec(25);
	}
	pincherDrive(0);
}

void midPincher()
{
	int indexR;
	int indexL;
	clearTimer(T2);
	while((time1(T2)<700) && (((pincherL.mid-5)<SensorValue[pincherL.sensorPort] <(pincherL.mid+5))  || ((pincherR.mid-5)<SensorValue[pincherR.sensorPort] <(pincherR.mid+5))))
	{
    if(SensorValue[pincherR.sensorPort]<(pincherR.mid-5))
    {
    	indexR = pCalc(pincherR.mid, pincherR.sensorPort,0.1);
      motor[pincherR.motorPort] = indexR;
    }
    else if(SensorValue[pincherR.sensorPort] > (pincherR.mid+5))
    {
    	indexR = pCalc(pincherR.mid, pincherR.sensorPort,0.1);
      motor[pincherR.motorPort] = - indexR;
    }
   	else
   		motor[pincherR.motorPort] =0;

    if(SensorValue[pincherL.sensorPort]< (pincherL.mid-5))
    {
    	indexL = pCalc(pincherL.mid, pincherL.sensorPort, 0.1);
      motor[pincherL.motorPort] = indexL;
    }
    else if(SensorValue[pincherL.sensorPort] > (pincherL.mid+5))
    {
    	indexL = pCalc(pincherL.mid, pincherL.sensorPort, 0.1);
      motor[pincherL.motorPort] = - indexL;
    }
    else
    	motor[pincherL.motorPort] = 0;
	}
  pincherDrive(0);
}

void autoArmDown()
{
	while(abs(SensorValue[arm.sensorPort])>arm.down)
	{
		armDrive(-80);
	}
	armDrive(0);
}

void autoArmUp()
{
	while(abs(SensorValue[arm.sensorPort])<arm.full)
	{
		armDrive(125);
	}
	armDrive(0);
}

void autoArmHold(char type)
{
	int mv_armHold;
	switch(type)
	{
		case 0:
		mv_armHold = arm.holdNull;
		break;
		case 1:
	 	mv_armHold= arm.holdStar;
		break;
		case 2:
		mv_armHold = arm.holdCube;
		break;
		case 3:
		mv_armHold = arm.hold3Stars;
		break;
	}

	while(abs(SensorValue[arm.sensorPort])<mv_armHold)
	{
		armDrive(60);
		pincherDrive(15);
		wait1Msec(25);
	}
	armDrive(0);
}

void midArm()
{
	while(abs(SensorValue[arm.sensorPort])> 65)
	{
		armDrive(-70);
		wait1Msec(25);
	}
	armDrive(0);
}

void flashLED(int n)
{
	for(int i=0; i<n; i++)
	{
		bLCDBacklight=true;
		wait1Msec(500);
		bLCDBacklight=false;
		wait1Msec(500);
	}
}

task openPincher()
{
  while(SensorValue[pincherL.sensorPort] > pincherL.open  || SensorValue[pincherR.sensorPort] > pincherR.open)
	{
		int openIndexL = pCalc(pincherL.open, pincherL.sensorPort, 0.9);
		int openIndexR = pCalc(pincherR.open, pincherR.sensorPort,0.9);

		if(openIndexL < 20 || openIndexR <20)
		{
			break;
		}

		if(SensorValue[pincherL.sensorPort] > pincherL.open)
		{
			motor[pincherL.motorPort] = -openIndexL;
		}

		if(SensorValue[pincherR.sensorPort] > pincherR.open)
		{
			motor[pincherR] = -openIndexR;
		}
	}
	pincherDrive(0);
}

task closePincher()
{
	int speedL;
	int speedR;
  int preReadL = pincherL.open-50;//ensure that we have a big speed at start so it won't trigger then break
	int preReadR = pincherR.open-50;
	int currentReadL;
	int currentReadR;
	int achievedCountL =0;
	int achievedCountR =0;

	while(SensorValue[pincherL.sensorPort] < pincherL.close || SensorValue[pincherR.sensorPort] < pincherR.close)
	{
		currentReadL = SensorValue[pincherL.sensorPort];
		currentReadR = SensorValue[pincherR.sensorPort];

		speedL = abs(currentReadL - preReadL);
		speedR = abs(currentReadR - preReadR);

		if(SensorValue[pincherL.sensorPort] < pincherL.close)
		{
			if(speedL>1)
			{
				motor[pincherL.motorPort] =125;
			}
			else
			{
				achievedCountL++;
				motor[pincherL.motorPort] = 0;
			}
		}
		else
		{
			motor[pincherL.motorPort] = 0;
		}

		if(SensorValue[pincherR.sensorPort] < pincherR.close)
		{
			if(speedR>1)
			{
				motor[pincherR.motorPort] =125;
			}
			else
			{
				achievedCountR++;
				motor[pincherR.motorPort] = 0;
			}
		}
		else
		{
			motor[pincherR.motorPort] = 0;
		}

		if(achievedCountL >1 && achievedCountR >1)
		{
			break;
		}
		preReadL = currentReadL;
		preReadR = currentReadR;
		wait1Msec(25);
	}

	while(pincherR.ifHold)
	{
		pincherDrive(20);
	}

}

#endif
