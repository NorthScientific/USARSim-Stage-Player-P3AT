#include <iostream>
#include <libplayerc++/playerc++.h>

int
main(int argc, char *argv[])
{
using namespace PlayerCc;

PlayerClient robot1("128.2.176.119",6660);
Position2dProxy pp1(&robot1, 3);

PlayerClient robot2("128.2.176.119",6661);
Position2dProxy pp2(&robot2, 3);

pp1.SetMotorEnable (true);
pp2.SetMotorEnable (true);

pp1.GoTo( 5, -2, 0);
pp2.GoTo( 10, -2, 0);

double XPos, YPos, YawPos;
for (;;)
{
robot1.Read();
robot2.Read();

 XPos = pp1.GetXPos();
 YPos = pp1.GetYPos();
YawPos = pp1.GetYaw();

YawPos = (YawPos * 180) / (3.14);
 std::cout << "X Position: "<<XPos << " | Y Position: "<< YPos <<" | Yaw: "<< YawPos <<std::endl;

}

}


