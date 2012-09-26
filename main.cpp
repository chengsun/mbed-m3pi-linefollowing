#include "mbed.h"
#include "m3pi.h"

m3pi m;

#define MAXSPEED 0.84   // global scaling factor
#define FROM 200        // sensor reading that we regard as "completely black"
#define TO   800        // sensor reading that we regard as "completely white"

// magic constants which control how much each sensor influences wheel speed
const float sensorWeightL[] = {2, 2.2, 1.75, 1.1, -1};     // left wheel
const float sensorWeightR[] = {-1, 1.1, 1.75, 2.2, 2};     // right wheel

float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

int main() {
    while (true) {
        // get motor speeds from sensor values
        m._ser.putc(SEND_RAW_SENSOR_VALUES);
        float motorSpeedL = 0, motorSpeedR = 0;
        for (int i = 0; i < 5; ++i) {
            char lobyte = m._ser.getc(),
                 hibyte = m._ser.getc();
            short val = ((lobyte) | (hibyte<<8));
            motorSpeedL += (1 - clamp(float(val - TO) / (TO-FROM), 0, 1)) * sensorWeightL[i];
            motorSpeedR += (1 - clamp(float(val - TO) / (TO-FROM), 0, 1)) * sensorWeightR[i];
        }
        motorSpeedL /= 5;
        motorSpeedR /= 5;

        // this makes sure that if the sensors can't see anything at all,
        // the robot doesn't get stuck by staying where it is
        // (hopefully by moving it backwards it will find the line again)
        motorSpeedL -= 0.2;
        motorSpeedR -= 0.2;
        
        // perform movement
        m.left_motor (clamp(motorSpeedL * MAXSPEED, -0.2, 1));
        m.right_motor(clamp(motorSpeedR * MAXSPEED, -0.2, 1));
    }
}
