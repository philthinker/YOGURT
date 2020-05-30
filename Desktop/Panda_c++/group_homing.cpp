// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <franka/gripper.h>
int main()
{
	franka::Gripper gripper("172.16.0.2");
while(1)
{
	gripper.homing();
}
}
