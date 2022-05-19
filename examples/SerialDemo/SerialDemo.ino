#include <PCAL9535A.h>
#include <Arduino.h>

PCAL9535A IO;

char ReadArray[25] = {0};

void setup() {
	
	Serial.begin(38400);
	Serial.println("Welcome to the Expanse...");
	Serial.println(IO.begin());
}

void loop() {
	static int ReadLength = 0;
  	String ReadString;
	if(Serial.available() > 0) {
    char Input = Serial.read();

    // Increment counter while waiting for carrage return or newline
    // if(Input != 13 || Input != 10) {
    if(Input != '#' && Input != '!' && Input != '*') { //Wait for control, data, or SDI-12 line end
      ReadArray[ReadLength] = Input;
      ReadLength++;
    }

    if(Input == '#') { //Control line ending
		ReadString = String(ReadArray);
		ReadString.trim();
		memset(ReadArray, 0, sizeof(ReadArray));
		ReadLength = 0;
		int Pin = 0;
		int State = 0;
		int Result = 0; //Used to grab result of either setting of pin mode or response from a read
		// char Device = 'D'; //Specify default device 
		// if(ReadString.charAt(0) < '0' || ReadString.charAt(0) > '9') { //If first digit is not a number, assume providing extended set 
		// 	Device = ReadString.charAt(0); 
		// 	ReadString.remove(0, 1); //Clear the leading character so it can be read normally 
		// }
		Pin = (ReadString.substring(0,2)).toInt(); //Grab the pin to operate on
		String Operation = ReadString.substring(2,3); //Grab the middle char
		State = (ReadString.substring(3,4)).toInt(); //Grab the state to set
		

		// digitalWrite(I2C_EN, LOW); //Turn off external port connection
		if(Operation.equals("M")) { //if call is for pinmode setting
			// switch(Device) { 
			// 	case 'D':
			// 		// pinMode(Pin, State); //Use normal state logic 
			// 		if(State == 0) pinMode(Pin, OUTPUT);
   //        			if(State == 1) pinMode(Pin, INPUT);
   //        			Result = 0; //Set manually
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.pinMode(Pin, !State); //Use inverse of state to corespond with 1 = input, 0 = output
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.pinMode(Pin, !State); //Use inverse of state to corespond with 1 = input, 0 = output
			// 		break;
			// }
			if(State == 0) State = OUTPUT; //Correct for weird naming rational to be logical with input numbers
			else if(State == 1) State = INPUT;
			Result = IO.pinMode(Pin, State); 
		}

		if(Operation.equals("D")) { //If call is for drive strength adjustment
			// switch(Device) {
			// 	case 'D':
			// 		digitalRead(Pin);
			// 		Result = 0; //Set manually
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.digitalRead(Pin);
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.digitalRead(Pin);
			// 		break;
			// }
			Result = IO.setDriveStrength(Pin, State);
		}
		
		if(Operation.equals("R")) { //If call is for digital read
			// switch(Device) {
			// 	case 'D':
			// 		digitalRead(Pin);
			// 		Result = 0; //Set manually
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.digitalRead(Pin);
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.digitalRead(Pin);
			// 		break;
			// }
			Result = IO.digitalRead(Pin);
		}

		if(Operation.equals("W")) { //If call is for digital write
			// switch(Device) {
			// 	case 'D':
			// 		// digitalWrite(Pin, State);
			// 		if(State == 0) digitalWrite(Pin, LOW);
   //        			if(State == 1) digitalWrite(Pin, HIGH);
			// 		Result = State; //Set manually //DEBUG!
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.digitalWrite(Pin, State);
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.digitalWrite(Pin, State);
			// 		break;
			// }
			Result = IO.digitalWrite(Pin, State);
		}

		if(Operation.equals("E")) { //If call is for digital write
			// switch(Device) {
			// 	case 'D':
			// 		// digitalWrite(Pin, State);
			// 		if(State == 0) digitalWrite(Pin, LOW);
   //        			if(State == 1) digitalWrite(Pin, HIGH);
			// 		Result = State; //Set manually //DEBUG!
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.digitalWrite(Pin, State);
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.digitalWrite(Pin, State);
			// 		break;
			// }
			Result = IO.reportError();
			IO.clearError();
		}

		if(Operation.equals("I")) { //If call is for digital write
			// switch(Device) {
			// 	case 'D':
			// 		// digitalWrite(Pin, State);
			// 		if(State == 0) digitalWrite(Pin, LOW);
   //        			if(State == 1) digitalWrite(Pin, HIGH);
			// 		Result = State; //Set manually //DEBUG!
			// 		break;
			// 	case 'A':
			// 		Result = ioAlpha.digitalWrite(Pin, State);
			// 		break;
			// 	case 'B':
			// 		Result = ioBeta.digitalWrite(Pin, State);
			// 		break;
			// }
			Result = IO.setInterrupt(Pin, State);
		}

		if(Operation.equals("P")) { //If call is for digital write
			Result = IO.setInputPolarity(Pin, State);
		}

		if(Operation.equals("L")) { //If call is for digital write
			Result = IO.setLatch(Pin, State);
		}

		if(Operation.equals("N")) { //If call is for digital write
			Result = IO.getAllInterrupts(IntAge::CURRENT);
		}
		// digitalWrite(I2C_EN, HIGH); //Turn on external port connection
		// while(Serial.peek() != '\n'); //Wait for new command
		// String NewCommand = Serial.readStringUntil('\n');
		Serial.print(">");
		Serial.println(ReadString); //Echo back to serial monitor
		Serial.print(">");
		Serial.println(Result); //Return various result 
	}
	}
}

