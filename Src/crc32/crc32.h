/*
 * crc32.h
 *
 *  Created on: 22 mag 2017
 *      Author: Emanuele
 */

#ifndef CRC32_CRC32_H_
#define CRC32_CRC32_H_

#include <stdio.h>

uint32_t crc32(uint32_t crc, const char* buf, uint8_t len);

#endif /* CRC32_CRC32_H_ */
