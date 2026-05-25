#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"

#include <cmath>
#include <cstdio>

// -------------------------
// Circular buffer
// -------------------------
#define BUFFER_LENGTH 16
struct CircularBuffer {
    double data[BUFFER_LENGTH];
    int index = 0;
    int count = 0;

    void push(double value) {
        data[index] = value;
        index = (index + 1) % BUFFER_LENGTH;
        if(count < BUFFER_LENGTH) count++;
    }

    double mean() {
        if(count == 0) return 0.0;
        double sum = 0;
        for(int i = 0; i < count; i++) sum += data[i];
        return sum / count;
    }
};

// -------------------------
// Light detection
// -------------------------
bool detect_light(CircularBuffer &buffer, double current_value) {
    if(buffer.count < 8) return false; // pas assez de données

    double avg = buffer.mean();

    // Si la valeur dépasse la moyenne locale de plus de 0.3, c'est une lumière
    return (current_value - avg) > 0.3;
}

// -------------------------
// Main
// -------------------------
int main(int argc, char **argv){
    Pioneer robot(argc, argv);
    robot.init();

    CircularBuffer light_buffer;

    while(robot.step() != -1) {
        double light = robot.get_light_intensity();

        // Ajouter au buffer
        light_buffer.push(light);

        // Détection simple
        if(detect_light(light_buffer, light)) {
            printf("[Time %.2f] Ici il y a une light !\n", robot.get_time());
        }

        // Navigation (inchangée)
        double* ps_values = robot.get_proximity();
        double lws=0.0, rws=0.0;
        fsm(ps_values, lws, rws);
        robot.set_motors_velocity(lws, rws);

        // Logging et affichage
        double* imu = robot.get_imu();
        log_csv("example.csv", 6, robot.get_time(), light, imu[0], imu[1], imu[2]);
        printf("Light intensity: %.3f\n", light);
    }

    close_csv();
    return 0;
}