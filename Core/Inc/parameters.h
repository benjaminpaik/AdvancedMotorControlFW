#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#define NUM_PARAMETERS 6

typedef struct {
  /*0*/ long InfoNumParameters;
  /*1*/ long InfoSwChecksum;
  /*2*/ float InfoSwVersion;
  /*3*/ float PidFocPropGain;
  /*4*/ float PidFocIntGain;
  /*5*/ float PidFocLimit;
} PARAMETER;

extern PARAMETER P;

#endif /* PARAMETERS_H_ */
