/* 
 * File:   Common.c
 * Author: Chris Hajduk
 *
 * Created on September 4, 2015, 1:34 PM
 */


#include "Common.h"

// TODO: make me a real checksum!
char generatePMDataDMAChecksum1(void) {
    return 0xAA;
}
char generatePMDataDMAChecksum2(void) {
    return 0x55;
}

char generateAMDataDMACheckbyte(void){
    return 0xAB;
}

char generateAMDataChecksum(AMData* data){
    char checksum = 0;
    int i = 0;
    //Two checksums and padding = 3 bytes
    for (i = 0; i < sizeof(AMData) - 2; i++){
        checksum += ((char*)data)[i];
    }
    return checksum;
}

float getDistance(long double lat1, long double lon1, long double lat2, long double lon2){ //in meters
    long double dLat = deg2rad(lat2 - lat1);
    long double dLon = deg2rad(lon2 - lon1);

    float a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);

    if ((dLat >= 0 && dLon >=0)||(dLat < 0 && dLon < 0)){
        return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a))) * 1000;
    }
    else {
         return EARTH_RADIUS * (2 * atan2(sqrt(a),sqrt(1 - a))) * -1000;
    }
}

void constrain(int16_t* input, int16_t min, int16_t max) {
    if (*input < min) {
        *input = min;
    }
    else if (*input > max) {
        *input = max;
    }
}
