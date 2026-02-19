#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define MAX_FILENAME 64

#define NUM_MULTIPLEXERS 3
#define NUM_DATA_POINTS 500 // number of data points
#define NUM_CHANNELS 32     // number of channels

float arr[NUM_CHANNELS][NUM_DATA_POINTS];
float past_temps[10];
int thermal_runaways[10];

static float Calculate_Channel_Mean(uint8_t channel);
static float Get_Channel_Temp(uint8_t channel, uint8_t poll_number);
static float Voltage_to_Temperature(float voltage);
static bool Is_Thermal_Runaway(float data[], int n, int k);
static void Get_Data(uint8_t n);
static void Delay(int ms);
static void Save_Temp(int i);

static void Delay(int ms) { usleep(ms * 1000); }

// convert voltage to temperature based on pre-calculated regression equation
static float Voltage_to_Temperature(float voltage) {
  return (((-6.2994E-9f * voltage + 1.53618E-6f) * voltage - 0.0000569765f) *
              voltage -
          0.0116001f) *
             voltage +
         2.16332f;
}

// get the gradient of an array in units/seconds using least squares
bool Is_Thermal_Runaway(float data[], int n, int k) {
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

  for (int i = 0; i < n; i++) {
    float x = (float)i;
    float y = data[i];

    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }

  float denominator = (n * sumX2 - sumX * sumX) * 4e-6;
  if (denominator == 0)
    return 0;

  if (((n * sumXY - sumX * sumY) / denominator) > 0) {
    thermal_runaways[k % 10] = 1;
  }

  // if 8 of the past 10 gradients are increasing, probably thermal_runaways
  // should probably also have a temp check....
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += thermal_runaways[i];
  }

  if (sum > 8) {
    return true;
  }

  return false;
}

// reads in data from provided file
static void Get_Data(uint8_t n) {
  char file_name[MAX_FILENAME];

  snprintf(file_name, sizeof(file_name), "mux_%d_data.txt", n);

  FILE *f;
  int height, width, i, j;

  if ((f = fopen(file_name, "r")) == NULL) {
    printf("file error - 85");
    exit(1);
  }

  if (fscanf(f, "%d%d", &height, &width) != 2) {
    printf("file error - 90");
    exit(1);
  }

  if (height < 1 || height > NUM_CHANNELS || width < 1 ||
      width > NUM_DATA_POINTS)
    exit(1);

  for (j = 0; j < height; j++)
    for (i = 0; i < width; i++)
      if (fscanf(f, "%f", &arr[j][i]) != 1)
        exit(1);
  fclose(f);
}

// read in data from a given sensor
static float Get_Channel_Temp(uint8_t channel, uint8_t poll_number) {
  return arr[channel][poll_number];
}

// take the mean of every data point from a given channel
static float Calculate_Channel_Mean(uint8_t channel) {
  float total = 0.0f;

  for (int i = 0; i < NUM_DATA_POINTS; i++) {
    total += Get_Channel_Temp(channel, i);
  }

  return (total / (float)NUM_DATA_POINTS);
}

int main(void) {
  struct timeval start, end;
  long seconds, microseconds;
  double time_elapsed;

  gettimeofday(&start, NULL);

  char file_name[64];

  // loop to simulate the car running
  for (int k = 0; k < 10; k++) {
    // take the average temp of the entire system
    float total = 0;

    // in the actual program we'll use j to select the correct multiplexer
    for (int j = 0; j < NUM_MULTIPLEXERS; j++) {
      // 'read' data from each set of sensors
      Get_Data(j);

      // take the average temp of each mux
      float mux_total = 0;

      // iterate through the 32 channels on the multiplexer and get the mean of
      // the 500 data points
      for (int i = 0; i < NUM_CHANNELS; i++) {
        float mean = Calculate_Channel_Mean(i);
        mux_total += mean;

        printf("\n");
        printf("%10.1f", mean);
        Delay(3);
      }

      total += (mux_total / (float)32);

      // for each channel, take the random
      printf("\n\n------------------------\n\n");
    }

    // store the past 10 temperatures
    past_temps[k % 10] = (total / (float)3);

    printf("\n\n------------PAST TEMPS------------\n\n");
    for (int i = 0; i < 9; i++) {
      printf("%10.1f", past_temps[i]);
      printf("\n");
    }

    // check for increasing gradient, if increasing toggle flag in array
    if (Is_Thermal_Runaway(past_temps, 10, k)) {
      printf("runaway");
      exit(1);
    }
  }

  gettimeofday(&end, NULL);
  seconds = end.tv_sec - start.tv_sec;
  microseconds = end.tv_usec - start.tv_usec;
  time_elapsed = seconds + microseconds * 1e-6;

  printf("\nExecution time: %.6f seconds\n", time_elapsed);

  return 0;
}
