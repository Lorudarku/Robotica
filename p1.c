#pragma config(StandardModel, "EV3_REMBOT")

///// VARIABLES GLOBALES
int distUmbral = 50; //Distancia minima del sensor de ultrasonidos
int robotSpeed = 90; //Velocdidad del robot

/*
Se acerca a una pared y cuando está cerca reduce la velcidad de forma progresiva
*/
task acercarsePared(){
		setLEDColor(ledGreenFlash);
		while(true){
			int d = getUSDistance(sonarSensor);
			if (d > distUmbral) {
				if (getUSDistance(sonarSensor) <= 2*distUmbral){
					setLEDColor(ledOrangeFlash);
				} else {
				setLEDColor(ledGreenFlash);
			}
				setMotorSpeed(leftMotor, robotSpeed);
				setMotorSpeed(rightMotor, robotSpeed);
			}	else {
				setMotorSpeed(leftMotor, d);
				setMotorSpeed(rightMotor, d);
			}
	}
}

task acercarseLuz(){
	int luzUmbral, luz;
	luzUmbral = getColorAmbient(colorSensor)*1.4;
	while(true){
		luz = getColorAmbient(colorSensor);
		if(luz > luzUmbral){
			setMotorSpeed(leftMotor, 30);
			setMotorSpeed(rightMotor, 0;
			while (luz <= getColorAmbient(colorSensor)){
				luz = getColorAmbient(colorSensor);
			}
			setMotorSpeed(leftMotor, 0);
			setMotorSpeed(rightMotor, 30);
			while (luz <= getColorAmbient(colorSensor)){
				luz = getColorAmbient(colorSensor);
			}
		}
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
					setMotorSpeed(leftMotor, 30);
					setMotorSpeed(rightMotor, 0;
				}
			}
		abortTimeslice();
		}
}

//task dirigirse

task main()
{
	//startTask(acercarsePared);
	startTask(acercarseLuz);
	//startTask(girarIzquierda);

	while(true){
		abortTimeslice();
	}
}
