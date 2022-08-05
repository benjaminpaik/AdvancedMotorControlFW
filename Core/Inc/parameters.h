#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#define NUM_PARAMETERS 19

typedef struct {
  /*0*/ long InfoNumParameters;
  /*1*/ long InfoSwChecksum;
  /*2*/ float InfoSwVersion;
  /*3*/ long Reserved0;
  /*4*/ float Reserved1;
  /*5*/ long Reserved2;
  /*6*/ long Reserved3;
  /*7*/ long Reserved4;
  /*8*/ long Reserved5;
  /*9*/ long Reserved6;
  /*10*/ long Reserved7;
  /*11*/ long Reserved8;
  /*12*/ long Reserved9;
  /*13*/ long Reserved10;
  /*14*/ long Reserved11;
  /*15*/ long Reserved12;
  /*16*/ long Reserved13;
  /*17*/ long Reserved14;
  /*18*/ long Reserved15;
} PARAMETER;

extern PARAMETER P;

#endif /* PARAMETERS_H_ */
