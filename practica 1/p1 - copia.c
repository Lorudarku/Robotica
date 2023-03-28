#pragma config(StandardModel, "EV3_REMBOT")

///// VARIABLES GLOBALES
int distUmbral = 50; //Distancia minima del sensor de ultrasonidos
int robotSpeed = 90; //Velocdidad del robot
TSemaphore sem12, sem23, sem34;

/*
Se acerca a una pared y cuando está cerca reduce la velcidad de forma progresiva
*/
task acercarsePared(){
	setLEDColor(ledGreenFlash);
	while(true){
		int d = getUSDistance(sonarSensor);
		semaphoreLock(sem34); //Si no estamos haciendo ninguna de las otras tasks
		if(bDoesTaskOwnSemaphore(sem34)){
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
			semaphoreUnlock(sem34);
		}
	}
}

task acercarseLuz(){
	int luzUmbral, luz;
	luzUmbral = getColorAmbient(colorSensor)*1.4;
	while(true){
		luz = getColorName(colorSensor);
		semaphoreLock(sem12);
		if (bDoesTaskOwnSemaphore(sem12)){ //Si no se está escapando
			setLEDColor(ledOrange);
			if(bDoesTaskOwnSemaphore(sem23)) {semaphoreUnlock(sem23);} //Si siguiendo luz: no seguir
			if(luz == colorRed){
				semaphoreLock(sem23); //Seguir luz
				setMotorSpeed(leftMotor, 30);
				setMotorSpeed(rightMotor, 0);
				while (luz != colorRed){
					luz = getColorAmbient(colorSensor);
				}
				setMotorSpeed(leftMotor, 0);
				setMotorSpeed(rightMotor, 30);
				while (luz != colorRed){
					luz = getColorAmbient(colorSensor);
				}
				semaphoreUnlock(sem23); //Dejar seguir luz
			}
			semaphoreUnlock(sem12); //Dejar escapando
		} else {
			if(!bDoesTaskOwnSemaphore(sem23)){semaphoreLock(sem23);} //Si no siguiendo luz: seguir
		}
	}
}

/*

*/
task escapar(){
	while(true){
		int dist = getUSDistance(sonarSensor);

		if((getTouchValue(touchSensor)==1) || (dist<5)){
			semaphoreLock(sem12); //Estamos escapando
			if(bDoesTaskOwnSemaphore(sem12)){
				setLEDColor(ledRed); //led rojo
				setMotorSpeed(leftMotor, -20);
				setMotorSpeed(rightMotor, -20);
				sleep(1000);
				resetGyro(gyroSensor);
				repeatUntil(getGyroDegrees(gyroSensor) >=90)
				{
					setMotorSpeed(leftMotor, 20);
					setMotorSpeed(rightMotor, -20);
				}
				resetGyro(gyroSensor);

				semaphoreUnlock(sem12);
			}
		}
	}
}

task seguirPared(){
	bool pared = false;
	while(true){
		int dist = getUSDistance(sonarSensor);

		semaphoreLock(sem23);
		if (bDoesTaskOwnSemaphore(sem23)){ //Si no estamos sigiuiendo luz
			setLEDColor(ledGreen);

			if(bDoesTaskOwnSemaphore(sem34)){ semaphoreUnlock(sem34);}

			if (dist < 25){
				semaphoreLock(sem34); //Si estamos siguiendo pared
					pared=true;
					resetGyro(gyroSensor);
					repeatUntil(getGyroDegrees(gyroSensor) >= 135)
					{
						setMotorSpeed(leftMotor, 25);
						setMotorSpeed(rightMotor, -25);
					}
					resetGyro(S2);
				semaphoreUnlock(sem34);
			}

			if (dist>25 && pared){
				semaphoreLock(sem34);
					resetGyro(S2);
					repeatUntil(getGyroDegrees(S2) <= -5)
					{
						setMotorSpeed(leftMotor, 40);
						setMotorSpeed(rightMotor, 60);
					}
					resetGyro(S2);
				semaphoreUnlock(sem34);
			}
			semaphoreUnlock(sem23);
		}	else {
			if(!bDoesTaskOwnSemaphore(sem34)){semaphoreLock(sem34);}
		}
	}
}

task main()
{
	int luz;
	luz = getColorName(colorSensor);

	semaphoreInitialize(sem12);
	semaphoreInitialize(sem23);
	semaphoreInitialize(sem34);

	startTask(acercarsePared);
	startTask(seguirPared);
	startTask(acercarseLuz);
	startTask(escapar);

	while(true){
		luz=getColorName(colorSensor);
		if (luz == colorRed){
		stopAllTasks();
		}
	}
}
