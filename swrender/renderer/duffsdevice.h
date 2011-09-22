/*
Copyright (c) 2007, 2008 Markus Trenkwalder

All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.

* Neither the name of the library's copyright owner nor the names of its 
  contributors may be used to endorse or promote products derived from this 
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DUFFS_DEVICE_346D4123_8A0B_41e8_A0D0_86B9318204FA
#define DUFFS_DEVICE_346D4123_8A0B_41e8_A0D0_86B9318204FA

#if defined (_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#define DUFFS_DEVICE2(BEFORE_LOOP, LOOP_INSTRUCTION, LOOP_COUNT, AFTER_LOOP) \
{ \
	BEFORE_LOOP; \
	int __n = (LOOP_COUNT); \
	if (__n > 0) { \
		switch (__n & 1) { \
			case 0: do	{	LOOP_INSTRUCTION; \
			case 1: 		LOOP_INSTRUCTION; \
					} while ((__n -= 2) > 0); \
		} \
	} \
	AFTER_LOOP; \
}

#define DUFFS_DEVICE4(BEFORE_LOOP, LOOP_INSTRUCTION, LOOP_COUNT, AFTER_LOOP) \
{ \
	BEFORE_LOOP; \
	int __n = (LOOP_COUNT); \
	if (__n > 0) { \
		switch (__n & 3) { \
			case 0: do	{	LOOP_INSTRUCTION; \
			case 3: 		LOOP_INSTRUCTION; \
			case 2: 		LOOP_INSTRUCTION; \
			case 1: 		LOOP_INSTRUCTION; \
					} while ((__n -= 4) > 0); \
		} \
	} \
	AFTER_LOOP; \
}

#define DUFFS_DEVICE8(BEFORE_LOOP, LOOP_INSTRUCTION, LOOP_COUNT, AFTER_LOOP) \
{ \
	BEFORE_LOOP; \
	int __n = (LOOP_COUNT); \
	if (__n > 0) { \
		switch (__n & 7) { \
			case 0: do	{	LOOP_INSTRUCTION; \
			case 7: 		LOOP_INSTRUCTION; \
			case 6: 		LOOP_INSTRUCTION; \
			case 5: 		LOOP_INSTRUCTION; \
			case 4: 		LOOP_INSTRUCTION; \
			case 3: 		LOOP_INSTRUCTION; \
			case 2: 		LOOP_INSTRUCTION; \
			case 1: 		LOOP_INSTRUCTION; \
					} while ((__n -= 8) > 0); \
		} \
	} \
	AFTER_LOOP; \
}

#define DUFFS_DEVICE16(BEFORE_LOOP, LOOP_INSTRUCTION, LOOP_COUNT, AFTER_LOOP) \
{ \
	BEFORE_LOOP; \
	int __n = (LOOP_COUNT); \
	if (__n > 0) { \
		switch (__n & 15) { \
			case 0: do	{	LOOP_INSTRUCTION; \
			case 15: 		LOOP_INSTRUCTION; \
			case 14: 		LOOP_INSTRUCTION; \
			case 13: 		LOOP_INSTRUCTION; \
			case 12: 		LOOP_INSTRUCTION; \
			case 11: 		LOOP_INSTRUCTION; \
			case 10: 		LOOP_INSTRUCTION; \
			case 9: 		LOOP_INSTRUCTION; \
			case 8: 		LOOP_INSTRUCTION; \
			case 7: 		LOOP_INSTRUCTION; \
			case 6: 		LOOP_INSTRUCTION; \
			case 5: 		LOOP_INSTRUCTION; \
			case 4: 		LOOP_INSTRUCTION; \
			case 3: 		LOOP_INSTRUCTION; \
			case 2: 		LOOP_INSTRUCTION; \
			case 1: 		LOOP_INSTRUCTION; \
					} while ((__n -= 16) > 0); \
		} \
	} \
	AFTER_LOOP; \
}
#define DUFFS_DEVICE32(BEFORE_LOOP, LOOP_INSTRUCTION, LOOP_COUNT, AFTER_LOOP) \
{ \
        BEFORE_LOOP; \
        int __n = (LOOP_COUNT); \
        if (__n > 0) { \
                switch (__n & 31) { \
                        case 0: do	{	LOOP_INSTRUCTION; \
                        case 31: 		LOOP_INSTRUCTION; \
                        case 30: 		LOOP_INSTRUCTION; \
                        case 29: 		LOOP_INSTRUCTION; \
                        case 28: 		LOOP_INSTRUCTION; \
                        case 27: 		LOOP_INSTRUCTION; \
                        case 26: 		LOOP_INSTRUCTION; \
                        case 25: 		LOOP_INSTRUCTION; \
                        case 24: 		LOOP_INSTRUCTION; \
                        case 23: 		LOOP_INSTRUCTION; \
                        case 22: 		LOOP_INSTRUCTION; \
                        case 21: 		LOOP_INSTRUCTION; \
                        case 20: 		LOOP_INSTRUCTION; \
                        case 19: 		LOOP_INSTRUCTION; \
                        case 18: 		LOOP_INSTRUCTION; \
                        case 17: 		LOOP_INSTRUCTION; \
                        case 16: 		LOOP_INSTRUCTION; \
                        case 15: 		LOOP_INSTRUCTION; \
                        case 14: 		LOOP_INSTRUCTION; \
                        case 13: 		LOOP_INSTRUCTION; \
                        case 12: 		LOOP_INSTRUCTION; \
                        case 11: 		LOOP_INSTRUCTION; \
                        case 10: 		LOOP_INSTRUCTION; \
                        case 9: 		LOOP_INSTRUCTION; \
                        case 8: 		LOOP_INSTRUCTION; \
                        case 7: 		LOOP_INSTRUCTION; \
                        case 6: 		LOOP_INSTRUCTION; \
                        case 5: 		LOOP_INSTRUCTION; \
                        case 4: 		LOOP_INSTRUCTION; \
                        case 3: 		LOOP_INSTRUCTION; \
                        case 2: 		LOOP_INSTRUCTION; \
                        case 1: 		LOOP_INSTRUCTION; \
                                        } while ((__n -= 32) > 0); \
                } \
        } \
        AFTER_LOOP; \
}
#endif
