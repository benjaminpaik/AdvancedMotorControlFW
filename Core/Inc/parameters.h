#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#define NUM_PARAMETERS 5

typedef struct {
  /*0*/ long InfoNumParameters;
  /*1*/ long InfoSwChecksum;
  /*2*/ float InfoSwVersion;
  /*3*/ long Reserved0;
  /*4*/ float Reserved1;
} PARAMETER;

extern PARAMETER P;

#endif /* PARAMETERS_H_ */
