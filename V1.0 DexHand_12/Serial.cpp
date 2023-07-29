/*   Copyright 2021 Feetech RC Model CO.,LTD
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*
 * Serial.cpp
 * ����ͨ�Žӿ�
 * ����: 2016.8.9
 * ����: ̷����
 */

#include "Serial.h"


CSerial::CSerial()
{
	IOTimeOut = 2;
	pSerial = NULL;
}


int CSerial::readSerial(unsigned char *nDat, int nLen)
{
	int Size = 0;
	int ComData;
	unsigned long t_begin = millis();
	unsigned long t_user;
	while(1){
		ComData = pSerial->read();
		if(ComData!=-1){
			if(nDat){
				nDat[Size] = ComData;
			}
			Size++;
			t_begin = millis();
		}
		if(Size>=nLen){
			break;
		}
		t_user = millis() - t_begin;
		if(t_user>IOTimeOut){
			break;
		}
	}
	return Size;
}

int CSerial::writeSerial(unsigned char *nDat, int nLen)
{
	if(nDat==NULL){
		return 0;
	}
	return pSerial->write(nDat, nLen);
}

int CSerial::writeSerial(unsigned char bDat)
{
	return pSerial->write(&bDat, 1);
}

void CSerial::flushSerial()
{
	while(pSerial->read()!=-1);
}
