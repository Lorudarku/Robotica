#pragma config(StandardModel, "EV3_REMBOT")

///// VARIABLES GLOBALES
int sensorMinDistance = 10; //Distancia minima del sensor de ultrasonidos
int robotSpeed = 20; //Velocdidad del robot

/*
Se acerca a una pared y cuando está cerca se para
*/
task acercarsePared(){
		setLEDColor(ledGreenFlash);
		while(true){
			if (getUSDistance(sonarSensor) > sensorMinDistance) {
				if (getUSDistance(sonarSensor) <= 2*sensorMinDistance){
					setLEDColor(ledOrangeFlash);
				} else {
				setLEDColor(ledGreenFlash);
			}
				setMotorSpeed(leftMotor, robotSpeed);
				setMotorSpeed(rightMotor, robotSpeed);

			}	else {
				stopAllMotors();
			}
			abortTimeslice();
	}
}

/*
Gira a la izquierda una cantidad fija de grados
*/
task girarIzquierda(){
		clearTimer(T1); //Reinicia el timer T1
		int grados = getGyroDegrees(gyroSensor);
		while(true){
			if (getGyroDegrees(gyroSensor) > grados-180) {
				if(time1[T1] > 3000) { //Si pasa +3seg sin alcanzar grados
					stopAllMotors();
				} else {
					int gradosDentro = getGyroDegrees(gyroSensor);
					setMotorSpeed(leftMotor, -robotSpeed);
					setMotorSpeed(rightMotor, robotSpeed);
				}
			}
		abortTimeslice();
		}
}

//task dirigirse

task main()
{
	startTask(acercarsePared);
	//startTask(girarIzquierda);

	while(true){
		abortTimeslice();
	}
}
