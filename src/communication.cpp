#include "communication.h"

Communication::Communication() : radio(CE_PIN, CSN_PIN) {
}

bool Communication::init() {
    if (!radio.begin()) {
        Serial.println("Radio hardware not responding!");
        return false;
    }
    
    // Configure radio for high reliability
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);  // Use channel 108 (2.508 GHz)
    radio.setPayloadSize(sizeof(ControlInputs));
    radio.enableAckPayload();
    radio.setRetries(5, 15);  // 5 retries, 15*250us delay between retries
    
    // Open pipes for communication
    radio.openWritingPipe(address[1]);
    radio.openReadingPipe(1, address[0]);
    
    // Start listening for incoming data
    radio.startListening();
    
    return true;
}

bool Communication::receiveControlInputs(ControlInputs& inputs) {
    static unsigned long lastReceiveTime = 0;
    const unsigned long TIMEOUT_MS = 500; // 500ms timeout
    
    if (radio.available()) {
        radio.read(&inputs, sizeof(ControlInputs));
        lastReceiveTime = millis();
        
        // Constrain input values
        inputs.throttle = constrain(inputs.throttle, 0.0f, 1.0f);
        inputs.roll = constrain(inputs.roll, -1.0f, 1.0f);
        inputs.pitch = constrain(inputs.pitch, -1.0f, 1.0f);
        inputs.yaw = constrain(inputs.yaw, -1.0f, 1.0f);
        inputs.targetAltitude = constrain(inputs.targetAltitude, 0.0f, 5.0f);
        
        return true;
    }
    
    // Check for timeout
    if (millis() - lastReceiveTime > TIMEOUT_MS) {
        Serial.println("Radio timeout detected!");
        return false;
    }
    
    return false;
}

bool Communication::sendTelemetry(const TelemetryData& telemetry) {
    radio.stopListening();
    bool success = radio.write(&telemetry, sizeof(TelemetryData));
    radio.startListening();
    return success;
}

bool Communication::isConnectionHealthy() const {
    return radio.isChipConnected() && !radio.failureDetected();
} 