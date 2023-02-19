/*
  Example of use of the FFT library to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2018 Enrique Condés and Ragnar Ranøyen Homb
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL A5
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2500; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

int maximumFreq = 0;
int minimumFreq = 0;
int bits[4] = {0,0,0,0};

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
  while(!Serial);

  callibration();
  
  Serial.println("Ready");
}

void loop()
{

  float x = findDominantFrequency();
  assignBits(x);
  
  for (int i = 0; i < 4; i++){
    Serial.print(bits[i]);
    Serial.println();
  }
}
int state = HIGH;
int cur_state = HIGH;
void callibration(){
  Serial.println("Please sing the lowest note you can comfortably sing and hold it for 10 seconds");
  delay(500);
  //state = digitalRead(7);
  //while (state == HIGH){
    //state = digitalRead(7);
  //}
  Serial.println("Start");
  for (int i = 0; i < 5; i++){
    minimumFreq += findDominantFrequency();
  }
  minimumFreq = minimumFreq/5;

  Serial.print("Your minimum frequency is ");
  Serial.println(minimumFreq);
  

  Serial.println("Please sing the highest note you can comfortably sing and hold it for 10 seconds");
  delay(500);
  //while (digitalRead(7) == HIGH){
    
  //}
  for (int i = 0; i < 5; i++){
    maximumFreq += findDominantFrequency();
  }
  maximumFreq /= 5;

  Serial.print("Your maximum frequency is ");
  Serial.println(maximumFreq);
  Serial.println("Callibrated! You may start humming instructions");
  //while (digitalRead(7) == HIGH){
    
  //}
   delay(500);
}

double findDominantFrequency(){
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
  //Serial.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //Serial.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println("Dominant frequency: ");
  Serial.println(x, 6); //Print out what frequency is the most dominant.
  //while(1); /* Run Once */
  delay(1000); /* Repeat after delay */
  return x;
  
}


void assignBits(double dominantFrequency){
  //map lowest value to 0 and highest value to F and the chop off the first 4 bits 
  int pitch = map(dominantFrequency, minimumFreq, maximumFreq, 0,15); //need to to figure out these values

  //convert to bits
  //int bits[4];

  for (int i = 3; i > -1; i--){
    if (pitch - pow(2,i) < 0){
      bits[3-i] = 0;
    }
    else{
      bits[3-i] = 1;
      pitch = pitch-pow(2,i);
    }
  }
  
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
