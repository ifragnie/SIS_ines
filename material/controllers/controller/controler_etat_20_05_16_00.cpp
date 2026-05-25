// Controller for the Pioneer robot

#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

// -------------------------
// Constants
// -------------------------
#define SIGNAL_LENGTH 256
#define FILTER_WINDOW 32

// -------------------------
// Light state enumeration
// -------------------------
enum LightState{
    STABLE,
    SLOW_FLUCT,
    FAST_FLUCT
};

// -------------------------
// Circular buffer structure
// -------------------------
struct CircularBuffer {
    double data[SIGNAL_LENGTH];
    int index = 0;
    int count = 0;

    void push(double value) {
        data[index] = value;
        index = (index + 1) % SIGNAL_LENGTH;
        if(count < SIGNAL_LENGTH) count++;
    }

    double& operator[](int i) {
        return data[i];
    }

    int size() const { return count; }
};

// -------------------------
// Lamp detection (intensity-based, no fixed threshold)
// -------------------------
bool detect_light(CircularBuffer &buffer) {
    if(buffer.size() < 16) return false; // Pas assez de données

    int window = 16;
    double local_mean = 0.0;

    for(int i = buffer.size() - window; i < buffer.size(); i++)
        local_mean += buffer[i];
    local_mean /= window;

    double stddev = 0.0;
    for(int i = buffer.size() - window; i < buffer.size(); i++)
        stddev += (buffer[i] - local_mean) * (buffer[i] - local_mean);
    stddev = sqrt(stddev / window);

    double variation = fabs(buffer[buffer.size()-1] - local_mean);
    double threshold = (stddev < 1e-3) ? 0.05 * local_mean : 2.0 * stddev;

    return variation > threshold;
}

// -------------------------
// FFT-based light analysis
// -------------------------
LightState analyze_light(CircularBuffer &buffer){
    double mean = 0.0;
    for(int i=0;i<buffer.size();i++) mean += buffer[i];
    mean /= buffer.size();

    double stddev = 0.0;
    for(int i=0;i<buffer.size();i++)
        stddev += (buffer[i]-mean)*(buffer[i]-mean);
    stddev = sqrt(stddev/buffer.size());

    // FFT uniquement si écart-type significatif
    if(stddev < 0.05) return STABLE;

    // FFT comme avant
    kiss_fft_cfg cfg = kiss_fft_alloc(SIGNAL_LENGTH, 0, NULL, NULL);
    kiss_fft_cpx fft_in[SIGNAL_LENGTH];
    kiss_fft_cpx fft_out[SIGNAL_LENGTH];

    for(int i=0;i<SIGNAL_LENGTH;i++){
        double local_mean = 0;
        int count=0;
        for(int j=-FILTER_WINDOW/2;j<=FILTER_WINDOW/2;j++){
            int idx=i+j;
            if(idx>=0 && idx<SIGNAL_LENGTH){
                local_mean += buffer[idx];
                count++;
            }
        }
        local_mean /= count;
        fft_in[i].r = buffer[i] - local_mean;
        fft_in[i].i = 0;
    }

    kiss_fft(cfg, fft_in, fft_out);
    double low_energy=0, high_energy=0, total_energy=0;
    int half = SIGNAL_LENGTH/2;
    for(int i=1;i<half;i++){
        double mag = fft_out[i].r*fft_out[i].r + fft_out[i].i*fft_out[i].i;
        total_energy += mag;
        if(i < half/8) low_energy += mag;
        else high_energy += mag;
    }
    free(cfg);

    if(total_energy < 1e-6) return STABLE;
    low_energy /= total_energy;
    high_energy /= total_energy;

    // Adaptation des seuils pour tes valeurs observées
    if(high_energy > 0.3) return FAST_FLUCT; // stroboscope rapide
    if(low_energy > 0.2) return SLOW_FLUCT; // flash lent
    return STABLE; // stable
}

// -------------------------
// Main function
// -------------------------
int main(int argc,char **argv){
    Pioneer robot(argc,argv);
    robot.init();

    std::string f_example = "example.csv";
    int f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,");

    CircularBuffer light_buffer;

    while(robot.step()!=-1){
        double time = robot.get_time();
        double* ps_values = robot.get_proximity();
        double* wheel_rot = robot.get_encoders();
        double light = robot.get_light_intensity();
        double* imu = robot.get_imu();

        // Stocker la lumière dans le buffer circulaire
        light_buffer.push(light);

        // Détection lampe
        if(detect_light(light_buffer)){
            LightState state = analyze_light(light_buffer);

            switch(state){
                case STABLE:
                    printf("[Time %.2f] Light stable\n", time);
                    break;
                case SLOW_FLUCT:
                    printf("[Time %.2f] Light slow fluctuation\n", time);
                    break;
                case FAST_FLUCT:
                    printf("[Time %.2f] Light fast fluctuation\n", time);
                    break;
            }
        }

        // Navigation
        double lws=0.0, rws=0.0;
        fsm(ps_values, lws, rws);
        robot.set_motors_velocity(lws, rws);

        // Visualization
        robot.hide_state_estimate_marker();
        robot.set_state_estimate_marker(0.0, 0.0, time, time);

        // Logging
        log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);

        printf("Light intensity: %.6f\n", light);
        (void)wheel_rot;
    }

    close_csv();
    return 0;
}