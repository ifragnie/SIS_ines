#include <math.h>
#include "kiss_fft/kiss_fft.h" // FFT library
#include <sys/stat.h> 
#include <fstream>
#include "utils/log_data.hpp"


// Function to check if a file exists
bool file_exists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

void save_fft_to_csv(double* mag, int signal_length, int fft_id) {
    // Open the CSV file in append mode
    std::ofstream csv_file("fft_output.csv", std::ios::app);

    if (csv_file.is_open()) {
        // If the file is empty, write the header
        if (csv_file.tellp() == 0) { // Check if the file is empty
            csv_file << "FFT_ID";
            for (int n = 0; n < signal_length; n++) {
                csv_file << ",Magnitude[" << n << "]";
            }
            csv_file << "\n";
        }

        // Write a single line for the FFT result
        csv_file << fft_id; // Write the FFT ID
        for (int n = 0; n < signal_length; n++) {
            csv_file << "," << mag[n]; // Write each magnitude value
        }
        csv_file << "\n"; // End the line for this FFT result

        // Close the file
        csv_file.close();
    } else {
        // Handle file opening error
        std::cerr << "Error: Unable to open file for writing.\n";
    }
}


/**
 * @brief KISS FFT USAGE EXAMPLE
 * @param signal_data pointer to the array containing the signal to be analyzed 
 */

std::vector<double> compute_fft_and_save(double* signal_data, int length, int fft_id){
    
    kiss_fft_cfg cfg = kiss_fft_alloc(length, 0, NULL, NULL);

    kiss_fft_cpx cx_in[length];
    kiss_fft_cpx cx_out[length];

    for(int n = 0; n<length; n++){
        cx_in[n].r = signal_data[n];
        cx_in[n].i = 0.0;
    }

    kiss_fft(cfg, cx_in, cx_out);

    std::vector<double> mag(length);

    for (int n = 0; n<length; n++){
        mag[n] = sqrt(cx_out[n].r *cx_out[n].r + cx_in[n].i*cx_in[n].i);
    }

    save_fft_to_csv(mag.data(), length, fft_id);

    free(cfg);

    return mag;
}

double analyze_fft_and_classify(const std::vector<double>& mag, int length){

    //1. Filtre Frequency
    const int START = 7;
    const int END = 255;

    std :: vector <double> filtered;

    for (int i = START; i < END && i < length; i++){
        filtered. push_back(mag[i]);
    }

    //safety
    if (filtered.size() < 10) return 0;

    //2. Metrics
    double max_val = filtered[0];
    double min_val = filtered[0];
    double delta = 0.0;

    //2.1 global delta check
    for (double v : filtered) {
            if (v > max_val) max_val = v;
            if (v < min_val) min_val = v;
        }
    
    delta = max_val - min_val;

    if (delta > 5.0)return 1;

    // 2.2 mid-band variation
    int mid_start = 100- START;
    int mid_end = END;

    std :: vector <double> filtered_2;

    for (int i = mid_start; i < mid_end && i < length; i++){
        filtered_2. push_back(mag[i]);
    }

    max_val = filtered_2[0];
    min_val = filtered_2[0];

    for (double v : filtered_2) {
            if (v > max_val) max_val = v;
            if (v < min_val) min_val = v;
        }
    delta = max_val - min_val;

    if (delta > 0.15) return 2;

    return 3;
}

void light_analysis(Pioneer& robot, double axe_x, double axe_y){
    //Constante
    const double ALPHA_LIGHT = 0.04;
    const int WINDOW_SIZE = 512;
    const int PEAK_NEIGHBORS = 250;
    static double no_light = 0.0;

    static std::string f_light = "light.csv";
    static int f_light_cols = init_csv(f_light,"no_light, response_fft, x, y,");

    //Get actual values
    double time = robot.get_time();
    double intensity = robot.get_light_intensity();    

    //Store old location
    static double old_location[WINDOW_SIZE/2 + 2][2];

    for (int i = 0; i < WINDOW_SIZE/2 + 1; i++) {
        old_location[i][0] = old_location[i+1][0];
        old_location[i][1] = old_location[i+1][1];
    }
    old_location[WINDOW_SIZE/2 +1][0] = axe_x;
    old_location[WINDOW_SIZE/2+1][1] = axe_y;



    // =======================
    // SIGNAL SMOOTHING
    // =======================
    static double filtered_light = 0.0;

    // Filtre exponentiel (EMA)
    filtered_light = ALPHA_LIGHT * intensity + (1.0 - ALPHA_LIGHT) * filtered_light;

    // =======================
    // MOVING WINDOW
    // =======================

    // Buffer to store light signal values
    static double light_signal_buffer[WINDOW_SIZE] = {0}; // Initialize buffer with zeros
    static int buffer_index = 0; // Initialize buffer index

    bool peak_detected = false;
    int center = WINDOW_SIZE/2;
    double candidate = 0.0;
    int fft_id = 0;

    // Remplissage initial
    if (buffer_index < WINDOW_SIZE) {
        light_signal_buffer[buffer_index] = filtered_light;
        buffer_index++;
    } else {

    // Décalage de la fenêtre
        for (int i = 0; i < WINDOW_SIZE - 1; i++) {
            light_signal_buffer[i] = light_signal_buffer[i + 1];
        }

        // Nouvelle valeur filtrée
        light_signal_buffer[WINDOW_SIZE - 1] = filtered_light;
    

        // =======================
        // PEAK DETECTION
        // =======================

        // Analyze center point

        candidate = light_signal_buffer[center];

        peak_detected = true;
    
        for (int k = 1; k <= PEAK_NEIGHBORS; k++){
            if (candidate <= light_signal_buffer[center -k] || candidate <= light_signal_buffer[center + k]){
                peak_detected = false;
                break;
            }
        }
    }
    if (peak_detected) {
    
    no_light ++;

    std::vector<double> mag = compute_fft_and_save(light_signal_buffer, WINDOW_SIZE, fft_id);

    double response_fft = analyze_fft_and_classify(mag, WINDOW_SIZE);

    if(response_fft == 1){
        printf("Detected light n°%.1f, Status : Defect n°2 - Low Flashing, location :(%.3f,%.3f)[m]\n", no_light, old_location[0][0], old_location[0][1]);
    }else if(response_fft == 2){
        printf("Detected light n°%.1f, Status : Defect n°1 - Fast Flashing, location :(%.3f,%.3f)[m]\n", no_light, old_location[0][0], old_location[0][1]);
    }else{
        printf("Detected light n°%.1f, Status : Good, location :(%.3f,%.3f)[m]\n", no_light, old_location[0][0], old_location[0][1]);
    }

    fft_id++;

    log_csv(
        f_light,
        f_light_cols,
        no_light,
        response_fft,
        old_location[0][0], 
        old_location[0][1]
      );
    }
}