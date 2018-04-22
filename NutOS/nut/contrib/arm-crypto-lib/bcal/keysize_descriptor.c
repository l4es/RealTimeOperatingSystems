/* keysize_descriptor.c */
/*
    This file is part of the ARM-Crypto-Lib.
    Copyright (C) 2009  Daniel Otte (daniel.otte@rub.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
 * \file    keysize_descriptor.c 
 * \author  Daniel Otte
 * \email   daniel.otte@rub.de
 * \date    2009-01-07
 * \license GPLv3 or later
 */

#include <stdint.h>
#include <stdlib.h>
#include <crypto/keysize_descriptor.h>

#define KS8 ((uint8_t*)ks_desc)
uint8_t is_valid_keysize_P(const void* ks_desc, uint16_t keysize){
	uint8_t type;
	type = KS8[0];
	ks_desc = KS8 + 1;
	if(type==KS_TYPE_TERMINATOR)
		return 0;
	if(type==KS_TYPE_LIST){
		uint8_t items;
		uint16_t item;
		items = KS8[0];
		ks_desc = KS8 + 1;
		while(items--){
			item = KS8[1] * 256 + KS8[0];
			ks_desc = KS8 + 2;
			if(item==keysize)
				return 1;
		}
		ks_desc = KS8 - 2;
	}
	if(type==KS_TYPE_RANGE){
		uint16_t max, min;
		min = KS8[1] * 256 + KS8[0];
		ks_desc = (uint8_t*)ks_desc + 2;
		max = KS8[1] * 256 + KS8[0];
		if(min <= keysize && keysize <= max)
			return 1;
	}
	if(type==KS_TYPE_ARG_RANGE){
		uint16_t max, min, dist, offset;
		min     = KS8[1] * 256 + KS8[0];
		max     = KS8[3] * 256 + KS8[2];
		dist    = KS8[5] * 256 + KS8[4];
		offset  = KS8[7] * 256 + KS8[6];
		ks_desc = KS8 + 6;
		if(min <= keysize && keysize <= max && (keysize % dist == offset))
			return 1;
	}
	if(type>KS_TYPE_ARG_RANGE){
		/* bad error, you may insert a big warning message here */
		return 0;
	}
	return is_valid_keysize_P(KS8+1, keysize); /* search the next record */
}

uint16_t get_keysize(const void* ks_desc){
	uint8_t type;
	uint16_t keysize;
	type = *((uint8_t*)ks_desc);
	if(type==KS_TYPE_LIST){
		ks_desc = (uint8_t*)ks_desc + 1;
	}
	ks_desc = (uint8_t*)ks_desc + 1;
	keysize = *((uint8_t*)ks_desc);
	return keysize;
}

uint16_t get_keysizes(const void* ks_desc, uint16_t** list){
	uint8_t type;
	uint16_t items;
	uint8_t i;
	type = *((uint8_t*)ks_desc);
	ks_desc = (uint8_t*)ks_desc + 1;
	if(type==KS_TYPE_LIST){
		items = *((uint8_t*)ks_desc);
		ks_desc = (uint8_t*)ks_desc + 1;
		if(!*list){
			*list = malloc(items*2);
			if(!*list){
				return 0;
			}
		}
		for(i=0; i<items; ++i){
			((uint16_t*)(*list))[i] = *((uint16_t*)ks_desc);
			ks_desc = (uint8_t*)ks_desc + 2;
		}
		return items;
	}
	if(type==KS_TYPE_ARG_RANGE){
		uint16_t min, max, distance, offset;
		min = *((uint16_t*)ks_desc);
		ks_desc = (uint8_t*)ks_desc + 2;
		max = *((uint16_t*)ks_desc);
		ks_desc = (uint8_t*)ks_desc + 2;
		distance = *((uint16_t*)ks_desc);
		ks_desc = (uint8_t*)ks_desc + 2;
		offset = *((uint16_t*)ks_desc);
		items = (max-min)/distance+1;
		if(min%distance!=offset){
			--items;
			min += (distance-(min%distance-offset))%distance;
		}
		if(!*list){
			*list = malloc(items*2);
			if(!*list){
				return 0;
			}
		}
		i=0;
		while(min<max){
			((uint16_t*)*list)[i++] = min;
			min += distance;
		}
		return i;
	}
	if(type==KS_TYPE_RANGE){
		uint16_t min, max, distance=8, offset=0;
		min = *((uint16_t*)ks_desc);
		ks_desc = (uint8_t*)ks_desc + 2;
		max = *((uint16_t*)ks_desc);
		items = (max-min)/distance+1;
		if(min%distance!=offset){
			--items;
			min += (distance-(min%distance-offset))%distance;
		}
		if(!*list){
			*list = malloc(items*2);
			if(!*list){
				return 0;
			}
		}
		i=0;
		while(min<max){
			((uint16_t*)*list)[i++] = min;
			min += distance;
		}
		return i;
	}
	return 0;
}
