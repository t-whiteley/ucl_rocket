#include "rocket_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define FREQ 10
#define DT 0.01


int main() {
    // PARAMS FOR MODEL - DEL FOR REAL ONE
    Data d0 = {0, 0, 0, DT};
    Data* d = &d0;
    // float seconds = 1.f / FREQ;
    // unsigned int micro = (unsigned int)(seconds * 1e6);

    // PARAMS FOR DATA VIS - DEL FOR REAL ONE
    FILE *csv = fopen("output.csv", "w");
    fprintf(csv, "Time,Acceleration,Velocity,Altitude\n");


    // setup
    // (TODO) add dynamic cd
    float t = 0;
    float A = 0.25;
    float cd = 0.01;


    while (1) {
        if (d->h < 0 && t > 2) {
            break;
        }
        
        // THIS IS ALL PARAMS FOR MODEL AND DATA VIS - DEL FOR REAL ONE
        // usleep(micro);
        // printf("t: %f, a: %f, v: %f, h: %f\n", t, d->a, d->v, d->h);
        d = generate_model_data(d, t, A, cd);
        t += DT;
        fprintf(csv, "%.4f,%.4f,%.4f,%.4f\n", t, d->a, d->v, d->h);
    }


    fclose(csv);
    return 0;
}