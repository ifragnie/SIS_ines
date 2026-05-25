#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"
#include "braitenberg.hpp"
#include "FSM.hpp"
#include <cstdio>
#include <algorithm>

#define BUFFER_LENGTH 8

struct CircularBuffer {
    double data[BUFFER_LENGTH];
    int index = 0;
    int count = 0;

    void push(double value) {
        data[index] = value;
        index = (index + 1) % BUFFER_LENGTH;
        if(count < BUFFER_LENGTH) count++hhh;
    }

    double mean() {
        if(count == 0) return 0;
        double sum = 0;
        for(int i=0;i<count;i++) sum += data[i];
        return sum/count;
    }
};

// -------------------------
// Peak-based light detection
// -------------------------
bool detect_light_peak(CircularBuffer &buffer, double current_value, bool &light_detected) {
    static double prev_value = 0;
    static bool in_light = false;
    static double max_value = 0;

    double baseline = buffer.mean();
    double threshold = 0.1;       // seuil pour déclencher
    double exit_threshold = 0.2; // seuil pour considérer la lumière partie

    if(!in_light) {
        // Entrée dans la lumière
        if(current_value - baseline > threshold) {
            in_light = true;
            max_value = current_value;
            light_detected = false;
        }
    } else {
        // Suivi du maximum
        if(current_value > max_value)
            max_value = current_value;

        // Pic atteint si le signal commence à redescendre
        if(current_value < max_value && !light_detected) {
            light_detected = true;
            in_light = false; // bloquer jusqu'à la prochaine lumière
            prev_value = current_value;
            return true;
        }

        // Sortie complète de la lumière
        if(current_value - baseline < exit_threshold) {
            in_light = false;
            light_detected = false;
            max_value = 0;
        }
    }

    prev_value = current_value;
    return false;
}

// -------------------------
// Main
// -------------------------
int main(int argc, char **argv){
    Pioneer robot(argc, argv);
    robot.init();

    CircularBuffer light_buffer;
    bool light_detected = false;

    while(robot.step() != -1) {
        double light = robot.get_light_intensity();
        light_buffer.push(light);

        if(detect_light_peak(light_buffer, light, light_detected)) {
            printf("[Time %.2f] Ici il y a une light !\n", robot.get_time());
        }

        // Navigation simples
        double* ps_values = robot.get_proximity();
        double lws=0.0, rws=0.0;
        fsm(ps_values, lws, rws);
        robot.set_motors_velocity(lws, rws);

        // Logging
        double* imu = robot.get_imu();
        log_csv("example.csv", 6, robot.get_time(), light, imu[0], imu[1], imu[2]);
        printf("Light intensity: %.3f\n", light);
    }

    close_csv();
    return 0;
}