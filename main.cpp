#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cctype>
#include <cmath>
#include <regex>

// Constants
#define SERIAL_PORT "/dev/ttyAMA3"
#define BAUDRATE B9600
#define OUTPUT_FILE "/home/ubuntu/check/gps.txt"
#define AI_TRIGGER_FILE "/home/ubuntu/check/ai.txt"

// Split string by delimiter
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Convert NMEA to decimal degrees
double convertToDecimalDegrees(const std::string& raw, const std::string& direction) {
    if (raw.empty()) return 0.0;
    
    double ddmm = std::stod(raw);
    int degrees = static_cast<int>(ddmm / 100);
    double minutes = ddmm - (degrees * 100);
    double decimal = degrees + minutes / 60.0;
    
    if (direction == "S" || direction == "W")
        decimal = -decimal;
    
    return decimal;
}

// Extract distance from AI file message
double extractDistanceFromAIFile() {
    std::ifstream ai_file(AI_TRIGGER_FILE);
    if (!ai_file.is_open()) {
        return 0.0;
    }

    std::string line;
    std::getline(ai_file, line);
    ai_file.close();

    // Regex pattern to match "Avg Depth: X.XX meters"
    std::regex pattern(R"(Avg Depth:\s*([0-9]+(?:\.[0-9]+)?)\s*meters)");
    std::smatch matches;
    
    if (std::regex_search(line, matches, pattern)) {
        if (matches.size() > 1) {
            try {
                return std::stod(matches[1].str());
            } catch (...) {
                std::cerr << "Error parsing distance value" << std::endl;
            }
        }
    }
    return 0.0;
}

// Adjust coordinates by distance (default bearing = 0° North)
void adjustCoordinates(double& lat, double& lon, double distance, double bearing = 0.0) {
    const double R = 6378137.0; // Earth's radius in meters
    double bearing_rad = bearing * M_PI / 180.0;
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double new_lat_rad = lat_rad + (distance / R) * cos(bearing_rad);
    double new_lon_rad = lon_rad + (distance / R) * sin(bearing_rad) / cos(lat_rad);

    lat = new_lat_rad * 180.0 / M_PI;
    lon = new_lon_rad * 180.0 / M_PI;
}

// Run script in thread
void runScript(const std::string& command) {
    system(command.c_str());
}

int main() {
    // Open serial port
    int serial_fd = open(SERIAL_PORT, O_RDONLY | O_NOCTTY);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port " << SERIAL_PORT << std::endl;
        return 1;
    }

    // Configure UART
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(serial_fd, TCSANOW, &options);

    char buffer[256];
    std::string line;

    while (true) {
        ssize_t n = read(serial_fd, buffer, sizeof(buffer));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                char c = buffer[i];

                if (c == '\n') {
                    if (line.find("$GPGGA") == 0) {
                        std::vector<std::string> fields = split(line, ',');

                        if (fields.size() > 5) {
                            // Get original coordinates
                            double latitude = convertToDecimalDegrees(fields[2], fields[3]);
                            double longitude = convertToDecimalDegrees(fields[4], fields[5]);

                            // Check if AI file has content
                            std::ifstream ai_check(AI_TRIGGER_FILE);
                            if (ai_check.peek() != std::ifstream::traits_type::eof()) {
                                // Extract distance from AI file
                                double distance = extractDistanceFromAIFile();
                                if (distance > 0) {
                                    adjustCoordinates(latitude, longitude, distance);
                                    std::cout << "Adjusted coordinates by " << distance << " meters" << std::endl;
                                }
                            // Save coordinates (adjusted or original)
                            std::ofstream outfile(OUTPUT_FILE);
                            if (outfile.is_open()) {
                                outfile << latitude << ", " << longitude << std::endl;
                                outfile.close();
                                std::cout << latitude << ", " << longitude << std::endl;
                            }

                                // Run AI scripts
                                std::cout << "Running AI scripts..." << std::endl;
                                 // we need to put path of binary of this apps 
                                std::thread t1(runScript, "/home/ubuntu/check/a1.cpp");
                                std::thread t2(runScript, "/home/ubuntu/check/a2.cpp");
                                std::thread t3(runScript, "python3 /home/ubuntu/check/a3.py");
                                
                                t1.join();
                                t2.join();
                                t3.join();

                                // Clear trigger file
                                std::ofstream(AI_TRIGGER_FILE, std::ios::trunc).close();
                                std::ofstream(OUTPUT_FILE, std::ios::trunc).close();
                                std::cout << "Cleared both trigger and output files" << std::endl;                            }

                        }
                    }
                    line.clear();
                } else if (isprint(c)) {
                    line += c;
                }
            }
        }
    }

    close(serial_fd);
    return 0;
}