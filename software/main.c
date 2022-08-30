//socket includes
#include <netinet/in.h>
#include <sys/socket.h>
#define PORT 8080

//-------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>

#include "ydlidar_sdk.h"
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

#include "kobuki.h"

#define SPEED 100
#define ONLINE false

// #define WHEEL_ENCODER_TICKS_PER_ROTATION 370
#define WHEEL_ENCODER_TICKS_PER_ROTATION 1440
#define WHEEL_TRACK_IN_METERS 0.145
#define WHEEL_RADIUS_IN_METERS 0.036
#define DIST_TO_ENCODER_TICKS_CONSTANT WHEEL_ENCODER_TICKS_PER_ROTATION * WHEEL_TRACK_IN_METERS / (2 * M_PI * WHEEL_RADIUS_IN_METERS)
#define WHEEL_TRACK_CIRCUMFERENCE M_PI * WHEEL_TRACK_IN_METERS
#define TWO_PI M_PI * 2

struct packet {
    float timestamp;
    float distance;
    float angle;
    uint32_t left_encoder_ticks;
    uint32_t right_encoder_ticks;
};

//float sensor_data[5] = {0}; // timestamp, distance, angle, ticks left, ticks right

// theta in deg
int DegToEncoderTicks(float theta) {
    // // conversion from encoder ticks to meters
    // const float CONVERSION = 0.0006108;
    const float m_to_ticks = 1637.19711853; // [ticks/m]
    float dist =  (theta / 360.0) * WHEEL_TRACK_CIRCUMFERENCE;
    printf("arc length = %f", dist);
    return floorf(dist * m_to_ticks);
}

uint16_t getForwardEncoderTickDelta(uint16_t new_tick, uint16_t old_tick) {
    if (new_tick < old_tick) {
        return (0xFFFF) - old_tick + new_tick;
    } else {
        return new_tick - old_tick;
    }
}

uint16_t getBackwardEncoderTickDelta(uint16_t new_tick, uint16_t old_tick) {
    if (old_tick < new_tick) {
        return old_tick - new_tick;
    } else {
        return (0xFFFF) - new_tick + old_tick;
    }
}
float deg_to_rad(int deg){
    return deg * 0.01745329252;
}

// get the avg distance in 4 directions, out => [-1, 0, 90, 180, 270] 
//angles are in radian and range from -180 to 180
//didn't convert angles on the fly to save time
bool getAvg(LaserFan *scan, float *out){
    memset(out, 0, sizeof(out));
    out[0] = -1.0;
    int temp_cnt[5] = {0};
    for (int i = 0; i < scan->npoints; i++){
        if ( -0.174533 < scan->points[i].angle && scan->points[i].angle < 0.174533) { // Back
            out[1] += scan->points[i].range;
            temp_cnt[1] += 1;
        } else if (1.39626 < scan->points[i].angle && scan->points[i].angle < 1.74533) { //RIGHT
            out[2] += scan->points[i].range;
            temp_cnt[2] += 1;
        } else if (2.96706 < scan->points[i].angle && scan->points[i].angle <3.14159
                || -3.14159 < scan->points[i].angle && scan->points[i].angle< -2.96706) { //Front
            out[3] += scan->points[i].range;
            temp_cnt[3] += 1;
        } else if (-1.74533 < scan->points[i].angle && scan->points[i].angle< -1.39626 ){ // LEFT
            out[4] += scan->points[i].range;
            temp_cnt[4]+=1;
        }
    }
    for (int i = 1; i < 4; i++){
        out[i] = out[i] / temp_cnt[i];
    }
    // memset(out, 0, sizeof(out));
    // out[0] = -1;
    // int temp_cnt[5] = {0};
    // for (int i = 0; i < scan->npoints; i++){
    //     if (-0.523599< scan->points[i].angle && scan->points[i].angle< 0.523599){ //-30 to 30
    //         out[1] += scan->points[i].range;
    //         temp_cnt[1]+=1;
    //     } else if (1.0472 < scan->points[i].angle && scan->points[i].angle< 2.0944){ // 60 to 120
    //         out[2] += scan->points[i].range;
    //         temp_cnt[2]+=1;
    //     } else if (2.61799 < scan->points[i].angle && scan->points[i].angle< 3.14159
    //             ||-3.14159 < scan->points[i].angle && scan->points[i].angle< -2.61799 ){ // 150 to 180 or -180 to -150
    //         out[3] += scan->points[i].range;
    //         temp_cnt[3]+=1;
    //     } else if (-2.0944 < scan->points[i].angle && scan->points[i].angle< -1.0472){ // -120 to -60
    //         out[4] += scan->points[i].range;
    //         temp_cnt[4]+=1;
    //     }
    // }
    // for (int i = 1; i <5; i++){
    //     out[i] = out[i] / temp_cnt[i];
    // }
    printf("front range : %f/n", out[1]);
    return out[1] > 1; 
}


int main(int argc, const char *argv[]) {   
    kobukiDriveDirect(0, 0);
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    if (ONLINE) {
        //--------------socket------------------------
        // perror("started");
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons( PORT );
        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
    }
    //------------LIDAR init--------------------

    os_init(); 
    
    YDLidar *laser = lidarCreate();
    //string prop
    char port[50] = "/dev/ydlidar";
    LidarPort ports;
    int size = lidarPortList(&ports);
    int i = 0;
    for(i = 0; i < size; i++) {
        printf("port: %s\n", ports.port[i].data);
        /// last port
        strcpy(port, ports.port[i].data);
    }
    setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
    strcpy(port, "");
    setlidaropt(laser, LidarPropIgnoreArray,port, sizeof(port));

    //int prop
    int i_optvalue = 115200;
    setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
     i_optvalue = TYPE_TOF;
    //i_optvalue = TYPE_TRIANGLE;
    setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
    i_optvalue = YDLIDAR_TYPE_SERIAL;
    setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
    i_optvalue = 3;
    setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));

    //bool prop
    bool b_optval = true;
    setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
    b_optval = true;
    setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
    b_optval = false;
    setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
    
    setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
    b_optval = true;
    setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
    b_optval = false;
    setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));

    //float prop
    float f_optval = 4.f;
    setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
    f_optval = 180.0f;
    setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
    f_optval = -180.0f;
    setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
    // f_optval = 8.f;
    f_optval = 10.f;
    setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
    // f_optval = 0.12f;
    f_optval = 0.12f;
    setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));

    getlidaropt(laser, LidarPropSerialBaudrate,&i_optvalue, sizeof(int));
    printf("baudrate: %d\n", i_optvalue);

    bool ret = initialize(laser);
    if(ret) {
        ret = turnOn(laser);
    }

    LaserFan scan;
    LaserFanInit(&scan);

    typedef enum {
        Stop,
        Scanning,
        Send_scan,
        Send_avg,
        Check,
        Receive,
        Choose_dir,
        Drive,
        Turn,
        Close
    } state_t;

    char direction = 'F';

    printf("------------started-----------------\n");
    KobukiSensors_t sensors;
    struct packet pkt;
    float avgs[5] = {0}; // -1.0, front avg, left avg, right avg, 0
    // float sensor_data[5] = {0}; // timestamp, distance, angle, ticks left, ticks right
    int alert[1] = {-2.0}; // message to notify command to give directions
    char dir[1] = {'A'}; // buff to get direction
    uint8_t buf_size = sizeof(char);
    bool good_range = true;
    
    // Drive Straight State Variables
    int16_t offset = 10;
    uint16_t prevLeftEncoder = 0;
    uint16_t prevRightEncoder = 0;
    uint8_t leftDiff = 0;
    uint8_t rightDiff = 0;
    int16_t rightVel = 100;
    int16_t leftVel = 100;
    
    // Turn State Variables
    int ticks = DegToEncoderTicks(90.0);
    uint32_t left = 0; // Number of Encoder ticks accumulated by the left wheel
    uint32_t right = 0; // Number of Encoder ticks accumulated by the right wheel
    uint32_t current_left = 0;
    uint32_t current_right = 0;
    uint32_t last_left = 0;
    uint32_t last_right = 0;
        
    state_t state = Scanning;
    while (1) {
        switch (state) {
            case Stop:
                // Initial state of romi.
                printf("Stopping...\n");
                kobukiDriveDirect(0, 0);
                sensors.leftWheelEncoder = 0;
                sensors.rightWheelEncoder = 0;
                if (read(new_socket, dir, buf_size) == buf_size) {
                    printf("Received: %c", *dir);
                    state = Scanning;
                } else {
                    kobukiDriveDirect(0, 0);
                    perror("receive failed");
                    exit(EXIT_FAILURE);
                }
                break;

            case Scanning: ;
                // Romi scans the environment using the lidar
                printf("SCANNING...\n");
                if(ret && os_isOk()) {
                    if(doProcessSimple(laser, &scan)) {
                        kobukiSensorPoll(&sensors);
                        good_range = getAvg(&scan, avgs);
                        state = Send_scan;
                    }
                } else {
                    fprintf(stderr, "Failed to get Lidar Data\n");
                    fflush(stderr);
                    state = Close;
                }
                break;

            case Send_scan: ;
                if (ONLINE){
                    printf("SENDING...\n");
                    // Romi sends lidar and wheel encoder data to command
                    pkt.timestamp = scan.stamp; // sensor_data[0] = scan.stamp;
                    pkt.left_encoder_ticks = sensors.leftWheelEncoder; // sensor_data[3] = sensors.leftWheelEncoder;
                    pkt.right_encoder_ticks = sensors.rightWheelEncoder; // sensor_data[4] = sensors.rightWheelEncoder;
                    // printf("(%d, %d, %d)\n", sensor_data[3], sensor_data[4], sizeof(sensors.leftWheelEncoder));
                    //printf("(%d, %d, %d)\n", pkt.left_encoder_ticks, pkt.right_encoder_ticks, sizeof(sensors.leftWheelEncoder));
                    
                    for (int i = 0; i < scan.npoints; i++) {
                        pkt.distance = scan.points[i].range; // sensor_data[1] = scan.points[i].range;
                        pkt.angle = scan.points[i].angle; // sensor_data[2] = scan.points[i].angle;
                        // if (send(new_socket, sensor_data, 5 * sizeof(float), 0) == -1) {
                        if (send(new_socket, &pkt, sizeof(struct packet), 0) == -1) {
                            kobukiDriveDirect(0, 0);
                            perror("send failed");
                            exit(EXIT_FAILURE);
                        }
                    }
                }    
                state = Send_avg;
                break;

            case Send_avg:
                if (ONLINE){
                    // Notify command that all sensor data has been sent and to publish data
                    // sensor_data[0] = -1.0;
                    pkt.timestamp = -1.0;
                    if (send(new_socket, &pkt, sizeof(struct packet), 0) == -1) {
                        kobukiDriveDirect(0, 0);
                        perror("send failed");
                        exit(EXIT_FAILURE);
                    }
                }
                // if (!good_range) {
                //     // sensor_data[0] = -2.0;
                //     pkt.timestamp = -2.0;
                //     if (send(new_socket, &pkt, sizeof(struct packet), 0) == -1) {
                //         kobukiDriveDirect(0, 0);
                //         perror("send failed");
                //         exit(EXIT_FAILURE);
                //     }
                // }
                // // Only go to driving state if front range is good
                state = Check;
                break;
                
            case Check: 

                printf("CHECKING FRONT RANGE...\n");
                // Romi checks if the front range is larger than 1.0
                // good_range = getAvg(&scan, avgs);
                if (!good_range) {
                    printf("--------WALL----------\n");
                    kobukiDriveDirect(0, 0);
                    if (ONLINE){
                        printf("------sending avg-----\n");
                        pkt.timestamp = -2.0; // send signal that I need a direction
                        if (send(new_socket, &pkt, sizeof(struct packet), 0) == -1) {
                            kobukiDriveDirect(0, 0);
                            perror("send failed");
                            exit(EXIT_FAILURE);
                        }
                        fflush(stdout);
                        state = Receive;
                    } else {
                        state = Choose_dir;
                    }
            
                } else {
                    state = Drive;
                }
                break;
                
            case Receive:
                printf("RECEIVING...\n");
                // Romi receives a direction from command
                if (read(new_socket, dir, buf_size) == buf_size) {
                    printf("Direction: %c \n", *dir);
                    direction = *dir;
                    state = Turn;
                    
                } else {
                    kobukiDriveDirect(0, 0);
                    perror("receive failed");
                    exit(EXIT_FAILURE);
                }
                break;
            case Choose_dir:
                printf("-------choosing Direction---------\n");
                // go to longest distance
                // avgs = [-1, 0, 90, 180, 270] 
                direction = avgs[2] > avgs[4] ? 'L' : 'R';
                state = Turn;
                break;

            case Drive:
                printf("DRIVING FORWARD...\n");
                // leftDiff = sensors.leftWheelEncoder - prevLeftEncoder;
                // rightDiff = sensors.rightWheelEncoder - prevRightEncoder;
                
                // prevLeftEncoder = sensors.leftWheelEncoder;
                // prevRightEncoder = sensors.rightWheelEncoder;
                
                
                // if (leftDiff > rightDiff) {
                //     rightVel = 150 + offset;
                //     leftVel = 150 - offset;
                // } else if (rightDiff > leftDiff) {
                //     rightVel = 150 - offset;
                //     leftVel = 150 + offset;
                // }
                
                //kobukiDriveDirect(leftVel, rightVel);
                kobukiDriveDirect(SPEED, SPEED + 10);
                state = Scanning;
                break;
            
            case Turn: 
                // Romi turns until the front range becomes large
                if (direction == 'R') {
                    printf("TURNING RIGHT...\n");
                    kobukiSensorPoll(&sensors);
                    left = 0; // Number of Encoder ticks accumulated by the left wheel
                    current_left = sensors.leftWheelEncoder;
                    last_left = sensors.leftWheelEncoder;
                    while (left  < ticks) {
                        //printf("ticks to turn: %d, progress: (%d, %d) [%d %d]\n", ticks, left, right, sensors.leftWheelEncoder, sensors.rightWheelEncoder);
                        kobukiDriveDirect(SPEED, -SPEED);
                        current_left = sensors.leftWheelEncoder;
                        left += getForwardEncoderTickDelta(current_left, last_left);
                        last_left = current_left;
                        kobukiSensorPoll(&sensors);
                    }
                    //printf("ticks to turn: %d, progress: (%d, %d) [%d %d]\n", ticks, left, right, sensors.leftWheelEncoder, sensors.rightWheelEncoder);
                } else if (direction == 'L') {
                    printf("TURNING LEFT...\n");
                    kobukiSensorPoll(&sensors);
                    right = 0; // Number of Encoder ticks accumulated by the right wheel
                    current_right = sensors.rightWheelEncoder;
                    last_right = sensors.rightWheelEncoder;
                    while (right  < ticks) {
                        //printf("ticks to turn: %d, progress: (%d, %d) [%d %d]\n", ticks, left, right, sensors.leftWheelEncoder, sensors.rightWheelEncoder);
                        kobukiDriveDirect(-SPEED, SPEED);
                        current_right = sensors.rightWheelEncoder;
                        right += getForwardEncoderTickDelta(current_right, last_right);
                        last_right = current_right;
                        kobukiSensorPoll(&sensors);
                    }
                    //printf("ticks to turn: %d, progress: (%d, %d) [%d %d]\n", ticks, left, right, sensors.leftWheelEncoder, sensors.rightWheelEncoder);
                }
                good_range = true;
                state = Drive;
                break;
                
            case Close:
                kobukiDriveDirect(0,0);
                LaserFanDestroy(&scan);
                turnOff(laser);
                disconnecting(laser);
                lidarDestroy(&laser);
                return 0;
        }
    }       
}
