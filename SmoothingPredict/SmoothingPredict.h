  /*
   *
	*
	*
	*
	*
	*/
	
	
float VelChangeDist(float desiredVel)  // returns distance required to effect the requested vel change
{
	const float J = 100.0f;
	const float A_max = 10.0f;
	
	float A_now = -10.0f;
	float V_now = -10.0f;
	float dP    = 0.0f;
	
	float t1   = (A_max - A_now) / J; // time to get from current accel to max accel
	Serial.print("t1 "); Serial.println(t1, 4);
	float t1_2 = t1 * t1;			// t1 squared
	float t1_3 = t1_2 * t1;			// t1 cubed
	
	float dV1 = 0.5f * J * t1_2 + A_now * t1; // change in velocity in stage 1
	
	
	float t3 = A_max / J;  // time to change from max accel to no accel
	
	float t3_2 = t3 * t3;		// t3 squared
	float t3_3 = t3_2 * t3;		// t3 cubed
	
	float dV3 = -0.5f * J * t3_2 + A_max * t3; // change in velocity in stage 3
	
	if(desiredVel > dV1 + dV3)  // max accel will be reached
	{
		float dV2 = (desiredVel - V_now) - (dV1 + dV3);	// vel change at max acceleration
		float t2  = dV2 / A_max;	// time at max acceleration
		Serial.print("t2 "); Serial.println(t2, 4);
		Serial.print("t3 "); Serial.println(t3, 4);
		float t2_2 = t2 * t2;
		
		Serial.println(dV1, 4);
		Serial.println(dV2, 4);
		Serial.println(dV3, 4);
		
		dP +=  0.166667f * J * t1_3 + 0.5f * A_now * t1_2 + V_now * t1;	// stage one: move to max accel
		Serial.println(dP, 4);
		dP +=  0.5f * A_max  * t2_2 + (V_now + dV1) * t2;						// stage two: complete max accel leg
		Serial.println(dP, 4);
		dP += -0.166667f * J * t3_3 + 0.5f * A_max * t3_2 + (V_now + dV1 + dV2) * t1;	// stage three: drop to zero accel
		Serial.println(dP, 4);
		
		return dP;
	}
	else	// case where max accel is not reached
	{
		Serial.println("Max Accel Not Reached");
		return 0.0f;
	}
	
}