task main()
{
  while(true) // Space between while (true) deleted
  {
    motor[chassisR]= vexRT[Ch2]*0.98;
    motor[chassisL] = vexRT[Ch3]*0.98;

    if(vexRT[Btn6U] ==1)
    {
      armDrive(30);
    }

    else if(vexRT[Btn6D]==1)
    {
      armDrive(-90);
    }

    else
    {
      armDrive(0);
    }

    if(vexRT[Btn5U] ==1)
    {
      pincherDrive(40);
    }

    else if(vexRT[Btn5D]==1)
    {
      pincherDrive(-40);
    }
    else
    {
      pincherDrive(0);
    }
  }
}
