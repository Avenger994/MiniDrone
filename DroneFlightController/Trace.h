
#ifndef _TRACE_
#define _TRACE_

#ifdef TRACE_ENABLED

#define START_TRACE(TraceName)                                          \
{ unsigned long TraceName##_EndTime = 0;                                \
unsigned long TraceName##_StartTimer = millis();


#define END_TRACE(TraceName)                                            \
TraceName##_EndTime = millis() - TraceName##_StartTimer;                \
Serial.println((String)#TraceName + " Time: " + TraceName##_EndTime + "ms"); }

#else

#define START_TRACE(TraceName){
#define END_TRACE(TraceName)}

#endif


#endif
