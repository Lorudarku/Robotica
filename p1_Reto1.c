#pragma config(StandardModel, "EV3_REMBOT")

///// VARIABLES GLOBALES
int sensorMinDistance = 10;
int robotSpeed = 20*5;


task main()
{
	while(1){
		setLEDColor(ledGreenFlash);

		while (getUSDistance(sonarSensor) > sensorMinDistance) {
			if (getUSDistance(sonarSensor) <= 2*sensorMinDistance){
				setLEDColor(ledOrangeFlash);
			}
			setMotorSpeed(leftMotor, robotSpeed);
			setMotorSpeed(rightMotor, robotSpeed);
		}

		clearTimer(T1); //Reinicia el timer T1
		int grados = getGyroDegrees(gyroSensor);
		while (getGyroDegrees(gyroSensor) > grados-180) {
			if(time1[T1] > 3000) { //Si pasa +3seg sin alcanzar grados
				stopAllMotors();
			} else {
				int gradosDentro = getGyroDegrees(gyroSensor);
				setMotorSpeed(leftMotor, -robotSpeed);
				setMotorSpeed(rightMotor, robotSpeed);
			}
		}
	}
}
