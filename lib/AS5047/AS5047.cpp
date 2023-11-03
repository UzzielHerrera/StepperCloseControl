#include <AS5047.h>

AS5047Spi::AS5047Spi(u8_t clockPin, u8_t misoPin, u8_t mosiPin, u8_t chipSelectPin) {
    // Initialize spi communication
    _clockPin = clockPin;
    _misoPin = misoPin;
    _mosiPin = mosiPin;
    _chipSelectPin = chipSelectPin;
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    
    hspi.begin(_clockPin, _misoPin, _mosiPin, _chipSelectPin);
}

void AS5047Spi::writeData(u16_t command, u16_t value){
    // Begin transaction
    hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    // Send command
    digitalWrite(_chipSelectPin, LOW);
    hspi.transfer16(command);
    digitalWrite(_chipSelectPin, HIGH);
    // delayMicroseconds(1);

    // Write data
    digitalWrite(_chipSelectPin, LOW);
    hspi.transfer16(value);
    digitalWrite(_chipSelectPin, HIGH);
    // End transaction
    hspi.endTransaction();
    // delayMicroseconds(1);

}

u16_t AS5047Spi::readData(u16_t command, u16_t nopCommand){
    // Begin transaction
    hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    // Send command
    digitalWrite(_chipSelectPin, LOW);
    hspi.transfer16(command);
    digitalWrite(_chipSelectPin, HIGH);
    // delayMicroseconds(1);

    // Read data
    digitalWrite(_chipSelectPin, LOW);
    uint16_t receivedData = hspi.transfer16(nopCommand);
    digitalWrite(_chipSelectPin, HIGH);

    // End transaction
    hspi.endTransaction();
    // delayMicroseconds(1);
    return receivedData;
}

AS5047::AS5047(u8_t clockPin, u8_t misoPin, u8_t mosiPin, u8_t chipSelectPin) : spi(clockPin, misoPin, mosiPin, chipSelectPin) {
    errorReading = false;
    sensor_position = 0;
    last_position = 0;
    sensor_position_with_rotations = 0;
    rotations_count = 0;
    motor_position_without_offset = 0;
    motor_position_steps = 0;
    offset = 0;
}

ReadDataFrame AS5047::readRegister(u16_t registerAddress){
    // Create command frame to transfer
    CommandFrame command;
    command.values.rw = READ;
    command.values.commandFrame = registerAddress;
    command.values.parc = isEven(command.raw);

    // Create nop command fram to transfer during data read
    CommandFrame nopCommand;
    nopCommand.values.rw = READ;
    nopCommand.values.commandFrame = NOP_REG;
    nopCommand.values.parc = isEven(nopCommand.raw);

    // Get read data frame from sensor
    ReadDataFrame receivedFrame;
    receivedFrame.raw = spi.readData(command.raw, nopCommand.raw);

    // Return read data frame
    return receivedFrame;
}

void AS5047::writeRegister(u16_t registerAddress, u16_t registerValue){
    // Create command frame to transfer
    CommandFrame command;
    command.values.rw = WRITE;
    command.values.commandFrame = registerAddress;
    command.values.parc = isEven(command.raw);

    // Create write data frame
    WriteDataFrame contentFrame;
    contentFrame.values.data = registerValue;
    contentFrame.values.low = 0;
    contentFrame.values.pard = isEven(contentFrame.raw);

    // Write data to sensor
    spi.writeData(command.raw, contentFrame.raw);
}

// Read uncompensated sensor angle in degrees
float AS5047::readAngleUncompensated(){
    ReadDataFrame readDataFrame = readRegister(ANGLE_REG);
    errorReading = readDataFrame.values.ef;
    Angleunc angleunc;
    angleunc.raw = readDataFrame.values.data;
    return angleunc.values.cordicang / 16384.0 * 360.0;
}

// Read uncompensated raw sensor angle in bits
u16_t AS5047::readRawAngleUncompensated(){
    ReadDataFrame readDataFrame = readRegister(ANGLE_REG);
    errorReading = readDataFrame.values.ef;
    Angleunc angleunc;
    angleunc.raw = readDataFrame.values.data;
    return angleunc.values.cordicang;
}

// Read compensated sensor angle in degrees
float AS5047::readAngleCompensated(){
    ReadDataFrame readDataFrame = readRegister(ANGLECOM_REG);
    errorReading = readDataFrame.values.ef;
    Anglecom anglecom;
    anglecom.raw = readDataFrame.values.data;
    return anglecom.values.daecang / 16384.0 * 360.0;
}

// Read compensated raw sendor angle in bit
u16_t AS5047::readRawAngleCompensated(){
    ReadDataFrame readDataFrame = readRegister(ANGLECOM_REG);
    errorReading = readDataFrame.values.ef;
    Anglecom anglecom;
    anglecom.raw = readDataFrame.values.data;
    return anglecom.values.daecang;
}

void AS5047::writeSettings1(Settings1 values) {
	writeRegister(SETTINGS1_REG, values.raw);
}

void AS5047::writeSettings2(Settings2 values){
	writeRegister(SETTINGS2_REG, values.raw);
}

void AS5047::writeZeroPosition(Zposm zposm, Zposl zposl){
	writeRegister(ZPOSM_REG, zposm.raw);
	writeRegister(ZPOSL_REG, zposl.raw);
}

//  Check if magnet too strong
bool AS5047::magnetTooStrong(){ 
    ReadDataFrame readDataFrame = readRegister(DIAGAGC_REG);
    errorReading = readDataFrame.values.ef;
    Diaagc diaagc;
    diaagc.raw = readDataFrame.values.data;
    return diaagc.values.magh;
}

// Check if magnet too weak
bool AS5047::magnetTooWeak(){
    ReadDataFrame readDataFrame = readRegister(DIAGAGC_REG);
    errorReading = readDataFrame.values.ef;
    Diaagc diaagc;
    diaagc.raw = readDataFrame.values.data;
    return diaagc.values.magl;
}

// Check if measure is realiable
bool AS5047::checkRealibility(){
    ReadDataFrame readDataFrame = readRegister(DIAGAGC_REG);
    errorReading = readDataFrame.values.ef;
    Diaagc diaagc;
    diaagc.raw = readDataFrame.values.data;
    return diaagc.values.cof;
}

// Read automatic gain control value for compensated angle measures
u16_t AS5047::getAutomaticGainControl(){
    ReadDataFrame readDataFrame = readRegister(DIAGAGC_REG);
    errorReading = readDataFrame.values.ef;
    Diaagc diaagc;
    diaagc.raw = readDataFrame.values.data;
    return diaagc.values.agc;
}

void AS5047::update_current_position(int microsteps){
    // Read sensor position bits
    sensor_position = readRawAngleCompensated();
    
    // Check sensor rotation
    if(sensor_position - last_position < - SENSOR_CPR_HALF){
        ++rotations_count;
    } else if (sensor_position - last_position > SENSOR_CPR_HALF){
        --rotations_count;
    }

    // Get total sensor position
    sensor_position_with_rotations = sensor_position + SENSOR_CPR_FULL * rotations_count;
    last_position = sensor_position;

    // Translate sensor to motor microsteps
    motor_position_without_offset = (sensor_position_with_rotations * microsteps * STEPPER_CPR) / SENSOR_CPR_FULL;
    motor_position_steps = motor_position_without_offset - offset;
}

// Set parity for data frames
bool AS5047::isEven(u16_t data){
    int count = 0;
    unsigned int b = 1;
    for(unsigned int i=0; i<15; i++){
        if(data & (b << 1)){
            count++;
        }
    }
    
    if(count % 2 == 0){
        return false;
    } else {
        return true;
    }
}