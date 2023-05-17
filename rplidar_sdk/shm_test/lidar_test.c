/*
run make in rplidar_sdk
run make in rplidar_sdk/shm_test

cd into rplidar_sdk/output/Linux/Release
./ultra_simple --channel --serial /dev/ttyUSB0 115200

cd into rplidar_sdk/shm_test
./lidar_test
*/

#include <stdio.h>
#include <sys/mman.h>        /* For shm_open and mmap */
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <unistd.h>          /* For close */
#include <string.h>
#include <stdlib.h>

struct lidar_data {
    float theta;
    float distance;
    int quality;
}lidar_data;

int main(int argc, char *argv[]) {
    const char* name = "SHARED_MEMORY";
    int shm_fd = shm_open(name, O_RDONLY, 0666);
    struct lidar_data *data = mmap(NULL, sizeof(struct lidar_data), PROT_READ, MAP_SHARED, shm_fd, 0);
    struct lidar_data *temp_data = malloc(sizeof(struct lidar_data));

    while (1) {
	memcpy(temp_data, data, sizeof(struct lidar_data));
        if (temp_data->quality > 10 && temp_data->distance < 401 && temp_data->distance > 0) {
            if (temp_data->theta > 337.5 || temp_data->theta < 22.5) {
                printf("THETA [%f] | DISTANCE [%f] | QUALITY [%d]\n", temp_data->theta, temp_data->distance, temp_data->quality);
            }
        }
    }


    munmap(NULL, sizeof(*data));
    shm_unlink(name);
    close(shm_fd);

    return (0);
}
