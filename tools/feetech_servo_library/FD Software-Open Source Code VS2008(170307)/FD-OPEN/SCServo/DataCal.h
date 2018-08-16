#pragma once
class DataCal
{
public:
	DataCal(void);
	~DataCal(void);
	static int GetMemAddres(void* END, void* START);
	static int scs2host(INT16 data, int NegativeBit = 10);
	static int host2scs (INT16 data, int NegativeBit = 10);
	static void SwapHex(UINT16 &d16);
	static void SwapHex(INT16 &d16);
};
