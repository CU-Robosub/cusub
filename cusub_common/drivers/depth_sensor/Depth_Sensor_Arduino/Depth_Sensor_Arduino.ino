#define SAMPLES_PER_PUBLISH 256
#define ZERO_READING_SAMPLES 4096

double zeroM = 0;
double nadc;
double volts;
double m;

void zeroSensor() {
 int i = 0;
 for(nadc=0; i<ZERO_READING_SAMPLES; i++) nadc += analogRead(0);
 nadc /= ZERO_READING_SAMPLES;
 
 zeroM = ((nadc*5/1024)-1.7)*14.3;
 nadc = 0;



}

void setup() {
 Serial.begin(115200);
}

void loop() {

// if a 0 is sent over serial, run zero depth function
 if (Serial.available()){
  int in = Serial.read();
  if(in==97){
    zeroSensor();
  }
 }

 for(int i=0; i<SAMPLES_PER_PUBLISH; i++){
  nadc += analogRead(0);
 }
 nadc /= SAMPLES_PER_PUBLISH;
 volts = nadc*5/1024; // converts nadc value to a voltage
 m = ((volts-1.7)*14.3) - zeroM; // converts voltage to depth; was *1.66 -1.76
 Serial.println(m);
 nadc = 0;
}

