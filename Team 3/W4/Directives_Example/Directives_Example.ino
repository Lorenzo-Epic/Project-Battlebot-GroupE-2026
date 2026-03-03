const int test1 = 123;
const int test2 = 345;
///these values are compiled and stored into processor memory

#define test3 123;
#define test4 456
///this is compiled in a way that does not use any memory by replacing the
///values, only works with replacing constants though


#define DEBUG

#ifdef DEBUG
  Serial.println("DEBUG");
#else
  Serial.println("NO DEBUG");
#endif

/// "#ifndef" is "if not defined"

///This only runs if the value is defined, can be good for triggering 
///certain functions that won't be used later

void setup(){
}

void loop(){
}
