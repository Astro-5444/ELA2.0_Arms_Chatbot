#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PWM 100  // Minimum safe PWM value
#define MAX_PWM 500  // Maximum safe PWM value
#define STEP_SIZE 5  // Smaller steps for smoother motion
#define BASE_DELAY 5  // Faster updates for smooth movement

int last_pwm_values[16] = {0};  // Store the last known values for all 16 channels

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(50);  // Set frequency to 50Hz for stepper motors
    Serial.println("ESP32 Ready - Waiting for Commands...");
}

void moveStepperSmooth(int channel, int target_pwm) {
    target_pwm = constrain(target_pwm, MIN_PWM, MAX_PWM);  // Limit PWM range

    if (last_pwm_values[channel] == target_pwm) {
        Serial.print("Ignoring duplicate target for channel "); Serial.println(channel);
        return;
    }

    int current_pwm = last_pwm_values[channel];

    Serial.print("Moving Channel "); Serial.print(channel);
    Serial.print(" from PWM "); Serial.print(current_pwm);
    Serial.print(" to PWM "); Serial.println(target_pwm);

    while (current_pwm != target_pwm) {
        if (current_pwm < target_pwm) {
            current_pwm += STEP_SIZE;
            if (current_pwm > target_pwm) current_pwm = target_pwm;
        } else {
            current_pwm -= STEP_SIZE;
            if (current_pwm < target_pwm) current_pwm = target_pwm;
        }

        pwm.setPWM(channel, 0, current_pwm);
        delay(BASE_DELAY);
    }

    last_pwm_values[channel] = target_pwm;  // Update the stored value
}

void loop() {
    if (Serial.available()) {
        Serial.flush();
        String command = Serial.readStringUntil('\n');

        int commaIndex = command.indexOf(',');
        if (commaIndex == -1) return;

        int channel = command.substring(0, commaIndex).toInt();
        int pwm_value = command.substring(commaIndex + 1).toInt();

        Serial.print("Received Command → Channel: "); Serial.print(channel);
        Serial.print(" | Target PWM: "); Serial.println(pwm_value);

        moveStepperSmooth(channel, pwm_value);
    }
}
