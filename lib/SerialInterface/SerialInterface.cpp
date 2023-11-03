#include "SerialInterface.h"

void serialInit(){
    SerialUSB.begin(115200);
}

void printSerialMenu(){
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("===== DELTA STEPPER CONTROLLER =====");
    SerialUSB.print("Firmware: ");
    SerialUSB.println(DELTA_STEPPER_CONTROLLER_VERSION_MAYOR + '.' + DELTA_STEPPER_CONTROLLER_VERSION_MINOR + '.' + DELTA_STEPPER_CONTROLLER_VERSION_PATCH);
    SerialUSB.print("Identifier: ");
    SerialUSB.println(MOTOR_UNIQUE_ID);
    SerialUSB.println("");
    SerialUSB.println("Main menu");
    SerialUSB.println("");
    SerialUSB.println(" s  -  step");
    SerialUSB.println(" d  -  dir");
    SerialUSB.println(" p  -  print angle");
    SerialUSB.println("");
    SerialUSB.println(" c  -  write new calibration table");
    SerialUSB.println(" e  -  check encoder diagnositics");
    SerialUSB.println(" q  -  parameter query");
    SerialUSB.println("");
    SerialUSB.println(" x  -  position mode");
    SerialUSB.println(" v  -  velocity mode");
    SerialUSB.println(" t  -  torque mode");
    SerialUSB.println("");
    SerialUSB.println(" y  -  enable control loop");
    SerialUSB.println(" n  -  disable control loop");
    SerialUSB.println(" r  -  enter new setpoint");
    SerialUSB.println("");
    SerialUSB.println(" j  -  step response");
    SerialUSB.println(" k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled");
    SerialUSB.println(" g  -  generate sine commutation table");
    SerialUSB.println(" m  -  print main menu");
    // SerialUSB.println(" f  -  get max loop frequency");
    SerialUSB.println("");
}

void serialCheck() {        //Monitors serial for commands.  Must be called in routinely in loop for serial interface to work.
    if (SerialUSB.available()) {

        char inChar = (char)SerialUSB.read();

        switch (inChar) {

        case 'p':             //print
            printAngle();
            break;

        case 's':             //step
            oneStep();
            printAngle();
            break;

        case 'd':             //dir
            if (dir) {
            dir = false;
            }
            else {
            dir = true;
            }
            break;

        case 'w':                //old command
            calibrate();           //cal routine
            break;
            
        case 'c':
            calibrate();           //cal routine
            break;        

        case 'e':
            // readEncoderDiagnostics();   //encoder error?
            break;

        case 'y':
            r = (readAngle()+(360.0 * wrap_count));          // hold the current position
            SerialUSB.print("New setpoint ");
            SerialUSB.println(r, 2);
            closeLoopEnable();      //enable closed loop
            break;

        case 'n':
            closeLoopDisable();      //disable closed loop
            ledcWrite(DRIVER_VREF1_CHANNEL, 0);     //set phase currents to zero
            ledcWrite(DRIVER_VREF2_CHANNEL, 0);                       
            break;

        case 'r':             //new setpoint
            SerialUSB.println("Enter setpoint:");
            while (SerialUSB.available() == 0)  {}
            r = SerialUSB.parseFloat();
            SerialUSB.println(r);
            break;

        case 'x':
            mode = 'x';           //position loop
            break;

        case 'v':
            mode = 'v';           //velocity loop
            break;

        case 't':
            mode = 't';           //torque loop
            break;

        case 'h':               //hybrid mode
            mode = 'h';
            break;

        case 'q':
            printParameters();     // prints copy-able parameters
            break;

        case 'a':             //anticogging
            antiCoggingCalibration();
            break;

        case 'k':
            printParametersEditMenu();
            break;
            
        case 'g':
            printSineGenerator();
            break;

        case 'm':
            printSerialMenu();
            break;
            
        case 'j':
            stepResponse();
            break;

        default:
            break;
        }
    }
}

void printAngle(){
    SerialUSB.print("stepNumber: ");
    SerialUSB.print(stepNumber, DEC);
    SerialUSB.print(" , ");
    //  SerialUSB.print(stepNumber * aps, DEC);
    //  SerialUSB.print(" , ");
    SerialUSB.print("Angle: ");
    SerialUSB.print(readAngle(), 2);
    SerialUSB.print(", raw encoder: ");
    SerialUSB.print(readRawAngle());
    SerialUSB.println();
}

void printParameters(){
    SerialUSB.println(' ');
    SerialUSB.println("----Current Parameters-----");
    SerialUSB.println(' ');
    SerialUSB.println(' ');

    SerialUSB.print("volatile float Fs = ");
    SerialUSB.print(Fs, DEC);
    SerialUSB.println(";  //Sample frequency in Hz");
    SerialUSB.println(' ');

    SerialUSB.print("volatile float pKp = ");
    SerialUSB.print(pKp, DEC);
    SerialUSB.println(";      //position mode PID vallues.");
    
    SerialUSB.print("volatile float pKi = ");
    SerialUSB.print(pKi, DEC);
    SerialUSB.println(";");

    SerialUSB.print("volatile float pKd = ");
    SerialUSB.print(pKd, DEC);
    SerialUSB.println(";");
    
    SerialUSB.print("volatile float pLPF = ");
    SerialUSB.print(pLPF, DEC);
    SerialUSB.println(";");

    SerialUSB.println(' ');

    SerialUSB.print("volatile float vKp = ");
    SerialUSB.print(vKp, DEC);
    SerialUSB.println(";      //velocity mode PID vallues.");

    SerialUSB.print("volatile float vKi = ");
    SerialUSB.print(vKi , DEC);
    SerialUSB.println(";");
    // SerialUSB.println(vKi * Fs, DEC);
    // SerialUSB.println(" / Fs;");

    SerialUSB.print("volatile float vKd = ");
    SerialUSB.print(vKd, DEC);
    SerialUSB.println(";");
    // SerialUSB.print(vKd / Fs);
    // SerialUSB.println(" * FS;");
    SerialUSB.print("volatile float vLPF = ");
    SerialUSB.print(vLPF, DEC);
    SerialUSB.println(";");

    SerialUSB.println("");
    SerialUSB.println("//This is the encoder lookup table (created by calibration routine)");
    SerialUSB.println("");
    
    SerialUSB.println("const float __attribute__((__aligned__(256))) lookup[16384] = {");
    for (int i = 0; i < 16384; i++) {
        SerialUSB.print(lookup[i]);
        SerialUSB.print(", ");
    }
    SerialUSB.println("");
    SerialUSB.println("};");
}

void printSineGenerator(){
    int temp;
    SerialUSB.println("");
    SerialUSB.println("The sineGen() function in Utils.cpp generates a sinusoidal commutation table.");
    SerialUSB.println("You can experiment with different commutation profiles by modifying this table.");
    SerialUSB.println("The below table should be copied into sine_1 in Parameters.cpp.");   
    SerialUSB.println("");
    delay(3000);
    SerialUSB.println("Printing sine look up table:...");
    SerialUSB.println("");
    for (int x = 0; x <= 3600; x++) {
        //temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.25))));
        temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.0))));
        SerialUSB.print(temp);
        SerialUSB.print(", ");  
    }
}

void printParametersEditMenu(){
    SerialUSB.println();
    SerialUSB.println("Edit parameters:");
    SerialUSB.println();
    SerialUSB.println("p ----- position loop");
    SerialUSB.println("v ----- velocity loop");
    SerialUSB.println("o ----- other");
    SerialUSB.println("q ----- quit");
    SerialUSB.println();

    while (SerialUSB.available() == 0)  {}
    char inChar2 = (char)SerialUSB.read();

    switch (inChar2) {
        case 'p':
            printParametersEditPositionControl();
            break;
        case 'v':
            printParametersEditVelocityControl();
            break;
        case 'o':
            printParametersEditOther();
            break;
        default:
            break;
    }
}

void printParametersEditPositionControl(){
    bool quit = false;
    while(!quit){
        SerialUSB.println("Edit position loop gains:");
        SerialUSB.println();
        SerialUSB.print("p ----- pKp = ");
        SerialUSB.println(pKp, DEC);
        SerialUSB.print("i ----- pKi = ");
        SerialUSB.println(pKi, DEC);
        SerialUSB.print("d ----- pKd = ");
        SerialUSB.println(pKd, DEC);
        SerialUSB.print("l----- LPF = ");
        SerialUSB.println(pLPF,DEC);
        SerialUSB.println("q ----- quit");
        SerialUSB.println();
        
        while (SerialUSB.available() == 0)  {}
        char inChar3 = (char)SerialUSB.read();
        
        switch (inChar3) {
        case 'p':
            SerialUSB.println("pKp = ?");
            while (SerialUSB.available() == 0)  {}
            pKp = SerialUSB.parseFloat();
            SerialUSB.print("new pKp = ");
            SerialUSB.println(pKp, DEC);
            SerialUSB.println("");
            break;
        case 'i':
            SerialUSB.println("pKi = ?");
            while (SerialUSB.available() == 0)  {}
            pKi = SerialUSB.parseFloat();
            SerialUSB.print("new pKi = ");
            SerialUSB.println(pKi, DEC);
            SerialUSB.println("");
            break;
        case 'd':
            SerialUSB.println("pKd = ?");
            while (SerialUSB.available() == 0)  {}
            pKd = SerialUSB.parseFloat();
            SerialUSB.print("new pKd = ");
            SerialUSB.println(pKd, DEC);
            SerialUSB.println("");
            break;
        case 'l':
            SerialUSB.println("pLPF = ?");
            while (SerialUSB.available() == 0)  {}
            pLPF = SerialUSB.parseFloat();
            pLPFa = exp(pLPF*-2*3.14159/Fs);
            pLPFb = (1.0-pLPFa);
            SerialUSB.print("new pLPF = ");
            SerialUSB.println(pLPF, DEC);
            SerialUSB.println("");
            break;
        case 'q':
            quit = true;
            SerialUSB.println("");
            SerialUSB.println("done...");
            SerialUSB.println("");
        default:
            break;
        }
    }
}

void printParametersEditVelocityControl(){
    bool quit = false;
    while(!quit){  
        SerialUSB.println("Edit velocity loop gains:");
        SerialUSB.println();
        SerialUSB.print("p ----- vKp = ");
        SerialUSB.println(vKp, DEC);
        SerialUSB.print("i ----- vKi = ");
        SerialUSB.println(vKi, DEC);
        SerialUSB.print("d ----- vKd = ");
        SerialUSB.println(vKd, DEC);
        SerialUSB.print("l ----- vLPF = ");
        SerialUSB.println(vLPF, DEC);
        SerialUSB.println("q ----- quit");
        SerialUSB.println();
    
        while (SerialUSB.available() == 0)  {}
        char inChar4 = (char)SerialUSB.read();
    
        switch (inChar4) {
        case 'p':
            SerialUSB.println("vKp = ?");
            while (SerialUSB.available() == 0)  {}
            vKp = SerialUSB.parseFloat();
            SerialUSB.print("new vKp = ");
            SerialUSB.println(vKp, DEC);
            break;
        case 'i':
            SerialUSB.println("vKi = ?");
            while (SerialUSB.available() == 0)  {}
            vKi = SerialUSB.parseFloat();
            SerialUSB.print("new vKi = ");
            SerialUSB.println(vKi, DEC);
            break;
        case 'd':
            SerialUSB.println("vKd = ?");
            while (SerialUSB.available() == 0)  {}
            vKd = SerialUSB.parseFloat();
            SerialUSB.print("new vKd = ");
            SerialUSB.println(vKd, DEC);
            break;
        case 'l':
            SerialUSB.println("vLPF = ?");
            while (SerialUSB.available() == 0)  {}
            vLPF = SerialUSB.parseFloat();
            vLPFa = (exp(vLPF*-2*3.14159/Fs));
            vLPFb = (1.0-vLPFa)* Fs * 0.16666667;
            SerialUSB.print("new vLPF = ");
            SerialUSB.println(vLPF, DEC);
            SerialUSB.println("");
            break;
        case 'q':
            quit = true;
            SerialUSB.println("");
            SerialUSB.println("done...");
            SerialUSB.println("");
        default:
            break;
        }
    }  
}

void printParametersEditOther(){
    SerialUSB.println("Edit other parameters:");
    SerialUSB.println();
    SerialUSB.print("p ----- PA = ");
    SerialUSB.println(PA, DEC);
    SerialUSB.println();

    while (SerialUSB.available() == 0)  {}
    char inChar3 = (char)SerialUSB.read();

    switch (inChar3) {
        case 'p':
            SerialUSB.println("PA = ?");
            while (SerialUSB.available() == 0)  {}
            PA = SerialUSB.parseFloat();
            SerialUSB.print("new PA = ");
            SerialUSB.println(PA, DEC);

            break;  
        default:
            break;
    }
}

void stepResponse(){
    SerialUSB.println("");
    SerialUSB.println("--------------------------------");
    SerialUSB.println("");
    SerialUSB.println("Get ready for step response!");
    SerialUSB.println("Close Serial Monitor and open Tools>>Serial Plotter");
    SerialUSB.println("You have 10 seconds...");
    closeLoopEnable();     //start in closed loop mode
    //mode = 'x';
    r = 0;
    delay(1000);
    SerialUSB.println("9...");
    delay(1000);
    SerialUSB.println("8...");
    delay(1000);
    SerialUSB.println("7...");
    delay(1000);
    SerialUSB.println("6...");
    delay(1000);
    SerialUSB.println("5...");
    delay(1000);
    SerialUSB.println("4...");
    delay(1000);
    SerialUSB.println("3...");
    delay(1000);
    SerialUSB.println("2...");
    delay(1000);
    SerialUSB.println("1...");
    delay(1000);
    print_yw = true;
    delay(100);
    r = 97.65;      /// choose step size as you like, 97.65 gives a nice plot since 97.65*1024 = 10,000
    delay(400);
    print_yw = false;
    r = 0;
    delay(500);
    closeLoopDisable();
}

void antiCoggingCalibration(){
    SerialUSB.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
    mode = 'x';
    r = lookup[1];
    closeLoopEnable();
    delay(1000);


    for (int i = 1; i < 657; i++) {
        r = lookup[i];
        SerialUSB.print(r, DEC);
        SerialUSB.print(" , ");
        delay(100);
        SerialUSB.println(u, DEC);
    }
    SerialUSB.println(" -----------------REVERSE!----------------");

    for (int i = 656; i > 0; i--) {
        r = lookup[i];
        SerialUSB.print(r, DEC);
        SerialUSB.print(" , ");
        delay(100);
        SerialUSB.println(u, DEC);
    }
    SerialUSB.println(" -----------------DONE!----------------");
    closeLoopDisable();
}