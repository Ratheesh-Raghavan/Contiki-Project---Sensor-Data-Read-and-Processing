/***********************************************************************************/
/*                                                                                 */
/* Sensor Data Read and Processing                                                 */
/*                                                                                 */
/* Written By: Ratheesh Raghavan                                                   */
/* Date: 23-11-2021                                                                */
/*                                                                                 */
/* Functionalities Implemented:                                                    */
/* - Storage - Read Freq at 2/Sec, FIFO buffer logic                               */
/* - Activity measurement after every 6 readings, categorization by Std Deviation  */
/* - Aggregation and Reporting                                                     */
/* - Advanced Feature: Linear Regression Analysis                                  */
/*                                                                                 */
/***********************************************************************************/
#include "contiki.h"
#include "dev/light-sensor.h"
#include "dev/sht11-sensor.h"
#include <stdio.h> // for printf(). 

/***********************************************************************************/
/* function to get the integer part of a floating point number */
int d1(float f) // integer part.
{
  return((int)f);
}
/***********************************************************************************/

/***********************************************************************************/
/* function to get the fractional part of a floating point number */
/* upto 3 positions */
unsigned int d2(float f) // fractional part.
{
  if (f>0)
    return(1000*(f-d1(f)));
  else
    return(1000*(d1(f)-f));
}
/***********************************************************************************/

/***********************************************************************************/
/* function to get temperature reading from sensor */
float getTemperature(void)
{
  // for simulation sky mote.
  int tempADC = sht11_sensor.value(SHT11_SENSOR_TEMP_SKYSIM);
  float temp_c = 0.04*tempADC-39.6; //skymote uses 12-bit ADC, or 0.04 resolution.

  // for XM1000 mote.
  //int tempADC = sht11_sensor.value(SHT11_SENSOR_TEMP);
  //float temp_c = 0.01*tempADC-39.6; // xm1000 uses 14-bit ADC, or 0.01 resolution.

  return temp_c;
}
/***********************************************************************************/

/***********************************************************************************/
/* function to get light intensity reading from sensor */
float getLight(void)
{
  float V_sensor = 1.5 * light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/4096;
                                     // ADC-12 uses 1.5V_REF.
  float I = V_sensor/100000;         // xm1000 uses 100kohm resistor.
  float light_lx = 0.625*1e6*I*1000; // convert from current to light intensity.

  return light_lx;
}
/***********************************************************************************/

/***********************************************************************************/
/* function to find the square root */
float sqrt(float S)
{
  float difference = 0.0;
  float error = 0.001;  // error tolerance.
  float x = 10.0;       // initial guess.
  int i;
  for (i=0; i<50; i++)  // looping 50 times.
  {
    x = 0.5 * (x + S/x);
    difference = x*x - S;
    if (difference<0) difference = -difference;
    if (difference<error) break; // the difference is deemed small enough.
  }
  return x;
}
/***********************************************************************************/

/***********************************************************************************/
/* function to print the elements of an array */
void printArray(char ArrName[5], float Arr[12], int ArrElementsCount)
{
  int i;
  printf("\n%s = [", ArrName);
  for (i=0;i<ArrElementsCount;i++)
  {
    printf("%d.%03u", d1(Arr[i]), d2(Arr[i]));
    if (i<(ArrElementsCount-1))
    {
      printf (", ");
    }
  }
  printf("]\n");
}
/***********************************************************************************/

/***********************************************************************************/
/* function to find the median */
float getMedian(float TS[144], int TSElementsCount)
{
  float intermed;
  float median;
  int i, j;
  
  // first sort the array.
  for (i=0;i<TSElementsCount-1;i++) 
  {
    for (j=i+1;j<=TSElementsCount-1;j++)
    {
      if (TS[i] > TS[j]) 
      {
        intermed = TS[i];
        TS[i] = TS[j];
        TS[j] = intermed;
      }
    }
  }

  // find median value from the sorted array.
  if (TSElementsCount % 2 == 0)
    median = (TS[(TSElementsCount/2)-1] + TS[(TSElementsCount/2)])/2;
  else
    median = TS[(int)(TSElementsCount/2)];

  return median;
}
/***********************************************************************************/ 

/*---------------------------------------------------------------------------*/
PROCESS(sensor_reading_process, "Sensor reading process");
AUTOSTART_PROCESSES(&sensor_reading_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_reading_process, ev, data)
{
  static struct etimer timer;

  static int readcount = 0; // varied from 1 to 12, once reaches 12 this will be reset to 1.
  static float B[12] = {0}; // this is the buffer to save light readings.
  static float T[12] = {0}; // this is the buffer to save temperature readings.
  
  static int k = 6; // this is the frequency of measurement and reporting.

  // below variables are for calculating standard deviation.
  static float Sum, Mean, SumofDistSquares, StdDev; 

  // below variables are for aggregation.
  static int AggrElementsCount; // this is the count of aggregated elements - 1, 3, or 12.
  static float X[12]; // this is the array for aggregated elements, declared with max 12.

  // below variables are for linear regression analysis.
  static float slopes[144] = {0};
  static int slopes_count;
  static float median_slope;
  static float offsets[12] = {0};
  static float median_offset;
  static float EstT[12]; // this is the estimated temperature vector.
                         
  static int i,j;

  PROCESS_BEGIN();
  etimer_set(&timer, CLOCK_CONF_SECOND/2); // timer setting to trigger 2 events per second.
                                           
  SENSORS_ACTIVATE(light_sensor);
  SENSORS_ACTIVATE(sht11_sensor);

  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(ev=PROCESS_EVENT_TIMER);

    float temp_c = getTemperature();
    float light_lx = getLight();
    
    //
    // setup the readcount for each cycle of 12 readings.
    // this will be used for the processing downstream.
    //
    if (readcount<12)
    {
      readcount++;
    }
    else
    {
      readcount = 1;
    }
    
    //
    // logic for FIFO buffers for Light and Temperature readings.
    //
    for (i=0;i<11;i++) 
    {
      B[i] = B[i+1];
      T[i] = T[i+1];
    }
    B[11] = light_lx;
    T[11] = temp_c;
    printf("Light: %d.%03u lx, ", d1(light_lx), d2(light_lx));
    printf("Temp: %d.%03u C\n", d1(temp_c), d2(temp_c));

    //
    // logic for activity measurement, aggregation and reporting.
    //
    if (readcount==k || readcount==2*k) // k is the frequency of measurement.
    {
      // calculate standard deviation.
      Sum = 0;
      for (i=0;i<12;i++)
      {
        Sum = Sum + B[i];
      }
      Mean = Sum/12;
      
      SumofDistSquares = 0;
      for (i=0;i<12;i++)
      {
        SumofDistSquares = SumofDistSquares + ((B[i]-Mean)*(B[i]-Mean));
      }
      StdDev = sqrt(SumofDistSquares);

      // logic for aggregation. 
      if (StdDev<100)
      {
        AggrElementsCount = 1;
      }
      else
      {
        if (StdDev<1000)
        {
          AggrElementsCount = 3;
        }
        else
        {
          AggrElementsCount = 12;
        }
      }

      if (AggrElementsCount==1)
      {
        X[0] = (B[0]+B[1]+B[2]+B[3]+B[4]+B[5]+B[6]+B[7]+B[8]+B[9]+B[10]+B[11])/12;
      }
      if (AggrElementsCount==3)
      {
        X[0] = (B[0]+B[1]+B[2]+B[3])/4;
        X[1] = (B[4]+B[5]+B[6]+B[7])/4;
        X[2] = (B[8]+B[9]+B[10]+B[11])/4;
      }
      if (AggrElementsCount==12)
      {
        for (i=0;i<12;i++)
        { 
          X[i] = B[i];
        }
      }
      
      // print the output of activity measurement, aggregation (reporting).
      printf("\nMeasurement and Reporting (Frequency = After every %d Sensor Data Reads)",
             k);

      printArray("B", B, 12);

      printf("StdDev = %d.%03u\n", d1(StdDev), d2(StdDev));
      
      switch (AggrElementsCount)
      {
        case 1:
          printf("Aggregation = 12-into-1");
          break;

        case 3:
          printf("Aggregation = 4-into-1");
          break;
        case 12:
          printf("Aggregation = 1-into-1 (No Aggregation)");
          break;
      }
      
      printArray("X", X, AggrElementsCount);
      printf("\n");    
    }
    
    //
    // logic for linear regression analysis.
    //
    if (readcount == 12)
    {
      slopes_count = 0;
      for (i=0;i<12;i++) 
      {
        for (j=i+1;j<12;j++) 
        {
          if (B[i] != B[j])
          {
            slopes[slopes_count++] = (T[j] - T[i]) / (B[j] - B[i]);
          }
        }
      }
      median_slope = getMedian(slopes,slopes_count);

      for (i=0;i<12;i++) 
      {
        offsets[i] = T[i] - median_slope * B[i];
      }
      median_offset = getMedian(offsets,12); 
      
      // derive the estimated temperature vector, 
      // values are calculated using the linear equation.
      for (i=0;i<12;i++)
      {
        EstT[i] = median_slope * B[i] + median_offset;
      }     

      // print the output of linear regression analysis.
      printf("Linear Regression Analysis by Theil-Sen Estimator Method");
      printf(" (Frequency = After every %d Sensor Data Reads)\n", 2*k);
      printf("Assumption: Temperature is dependent on Light\n");
      printf("Light Vector (Independent Vector) B: "); 
      printArray("B", B, 12);
      printf("Temperature Vector (Dependent Vector) T: ");
      printArray("T", T, 12);
      printf("Median Slope: %d.%03u\n", d1(median_slope), d2(median_slope));
      printf("Median Offset: %d.%03u\n", d1(median_offset), d2(median_offset));
      printf("Linear Equation: Temperature = %d.%03u + %d.%03u * Light\n", 
             d1(median_offset), d2(median_offset), d1(median_slope), d2(median_slope));
      printf("Estimated Temperature Vector EstT:");
      printArray("EstT", EstT, 12);
      printf("\n");
    }
 
    etimer_reset(&timer);
    
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
