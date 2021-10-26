/*
 * dsp.h
 *
 *  Created on: Apr 17, 2021
 *      Author: benja
 */

#ifndef SRC_DSP_H_
#define SRC_DSP_H_

#define BYTES_PER_INT       2
#define BYTES_PER_LONG      4
// convert a 32-bit integer into a byte array
#define LONG_TO_BYTES(B, L) (B[0] = ((L >> 24) & 0xFF)); \
                            (B[1] = ((L >> 16) & 0xFF)); \
                            (B[2] = ((L >> 8) & 0xFF)); \
                            (B[3] = ((L) & 0xFF))
// convert 4 bytes into a 32-bit integer
#define BYTES_TO_LONG(B)  (((int32_t)(B)[0] << 24) | ((int32_t)(B)[1] << 16) | ((int32_t)(B)[2] << 8) | ((int32_t)(B)[3]))
#define BYTE_TO_INT(B)    (((int16_t)(B)[0] << 8) | (int16_t)(B)[1])
// convert a 32-bit integer to a 32-bit float
#define FLOAT_BITS(X)    (*(float_t *)(&X))
#define INT_BITS(X)      (*(int32_t *)(&X))

#endif /* SRC_DSP_H_ */
