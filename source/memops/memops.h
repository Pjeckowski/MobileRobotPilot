#ifndef MEMOPS_H_
#define MEMOPS_H_

#include<String.h>

float getValFromBytes(uint8_t bytes[4]);
void getBytes(double data, uint8_t* bytes);

void getBytes(double data, uint8_t* bytes)
{
	memcpy(bytes, (uint8_t *)(&data), 4);
}

float getValFromBytes(uint8_t bytes[4])
{
	float val = 0;
	memcpy((uint8_t*)(&val), bytes, 4);
	return val;
}


#endif /* MEMOPS_H_ */
