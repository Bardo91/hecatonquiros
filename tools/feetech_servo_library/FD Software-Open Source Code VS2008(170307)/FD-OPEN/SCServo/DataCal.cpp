#include "stdafx.h"
#include "DataCal.h"

DataCal::DataCal(void)
{
}

DataCal::~DataCal(void)
{
}


int DataCal::GetMemAddres(void* END, void* START)
{
	UCHAR *MemEnd,*MemST;
	MemEnd=(UCHAR*)(END);
	MemST=(UCHAR*)(START);
	int add = MemEnd-MemST;
	return add;
}


int DataCal::scs2host(INT16 data, int NegativeBit)
{
	if(data&(1<<NegativeBit))
		data = -(data&~(1<<NegativeBit));
	return data;
}

int DataCal::host2scs (INT16 data, int NegativeBit)
{
	if(data<0)
		data = ((-data)|(1<<NegativeBit));
	return data;
}

void DataCal::SwapHex(UINT16 &d16)
{
	UINT8 d8L;
	d8L = (d16&0xff00)>>8;
	d16 <<= 8;
	d16 |= d8L;
}

void DataCal::SwapHex(INT16 &d16)
{
	UINT8 d8L;
	d8L = (d16&0xff00)>>8;
	d16 <<= 8;
	d16 |= d8L;
}
