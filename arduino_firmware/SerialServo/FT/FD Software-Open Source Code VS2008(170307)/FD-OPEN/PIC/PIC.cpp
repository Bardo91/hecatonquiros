// PIC.cpp : 实现文件
//

#include "stdafx.h"
#include "PIC.h"
#include <math.h>

CPIC::CPIC() : CWnd()
{
	Vx = (int)pow(2.0, VMINX);
	LineSel = BLACKSEL|REDSEL|GREENSEL|BLUESEL;
	Horiz = 0;
	Verti = 25;
}

CPIC::~CPIC()
{
}

void CPIC::DoDataExchange(CDataExchange* pDX)
{
	CWnd::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CPIC, CWnd)
	ON_WM_PAINT()
END_MESSAGE_MAP()


// CPIC 消息处理程序
void CPIC::OnPaint()
{
	CPaintDC dc(this);
	DrawPIC(&MemDC);
	dc.SetMapMode(MM_LOMETRIC);//设置映像模式
	dc.SetStretchBltMode(HALFTONE);
	dc.StretchBlt(0,0,MaxX,-MaxY,&MemDC,0,0,MaxX,MaxY,SRCCOPY); 
	CWnd::OnPaint();
}

void CPIC::DrawLine(CPntLst &LineList, CDC *pDC, COLORREF rgb)
{
	POSITION ps;
	short y;
	short x = MaxX+Horiz;

	if(LineList.IsEmpty()){
		return;
	}

	CPen Pen;
	Pen.CreatePen(PS_SOLID, 3, rgb);
    CPen *pOldPen = pDC->SelectObject(&Pen);

	pDC->MoveTo(x, MaxY-LineList.GetHead()-Verti);
	for(ps=LineList.GetHeadPosition(); ps; LineList.GetNext(ps)){
		y = MaxY-(int)(LineList.GetAt(ps)*VertP)-Verti;
		pDC->LineTo(x,y);
		x -= Vx;
	}
	pDC->SelectObject(&pOldPen);
	Pen.DeleteObject();
}

void CPIC::DrawPIC(CDC *pDC)
{
	ErasePic(pDC);
	DrawScale(pDC, BLACKCLO);
	if(LineSel&BLACKSEL){
		DrawLine(BlackLine, pDC, BLACKCLO);
	}
	if(LineSel&REDSEL){
		DrawLine(RedLine, pDC, REDCLO);
	}
	if(LineSel&GREENSEL){
		DrawLine(GreenLine, pDC, GREENCLO);
	}
	if(LineSel&BLUESEL){
		DrawLine(BlueLine, pDC, BLUECLO);
	}
	if(LineSel&YELLOWSEL){
		DrawLine(YellowLine, pDC, YELLOWCLO);
	}
}

void CPIC::AddLineDat(CPntLst &LineList, short y)
{
	if(LineList.GetCount()>=(MaxX>>VMINX)){
		LineList.RemoveTail();
	}
	LineList.AddHead(y);
}


void CPIC::AddBlackLineDat(short y)
{
	AddLineDat(BlackLine,y);
}

void CPIC::AddGreenLineDat(short y)
{
	AddLineDat(GreenLine,y);
}

void CPIC::AddRedLineDat(short y)
{
	AddLineDat(RedLine,y);
}

void CPIC::AddBlueLineDat(short y)
{
	AddLineDat(BlueLine,y);
}

void CPIC::AddYellowLineDat(short y)
{
	AddLineDat(YellowLine,y);
}

void CPIC::DrawScale(CDC *pDC, COLORREF rgb)
{
	CPen Pen;
	Pen.CreatePen(PS_DASH, 1, rgb);
	CPen *pOldPen = pDC->SelectObject(&Pen);

	CFont   font;
	font.CreatePointFont(FontSize, "Arial", pDC);
	pDC->SelectObject(&font);
	pDC->SetTextColor(rgb); 

	int LineY = 0;
	int TextY = 0;
	int X;
	CString str;
	while(LineY<MaxY){
		int pointy = (MaxY-LineY-Verti);
		pDC->MoveTo(0, pointy);
		pDC->LineTo(MaxX, pointy);
		str.Format("%d", TextVP*TextY);
		pDC->TextOut(0, pointy-50, str.GetBuffer(0));
		str.Format("%d", -1000+2*TextY);
		if(TextY<100)
			X = 80+75;
		else if(TextY<500)
			X = 80+55;
		else if(TextY==500)
			X = 75;
		else if(TextY<1000)
			X = 70+55;
		else
			X = 90+55;
		pDC->TextOut(MaxX-X, pointy-50,str.GetBuffer(0));
		LineY+=(int)(100*VertP);
		TextY+=100;
	}
	pDC->SelectObject(&pOldPen);
	Pen.DeleteObject();
	font.DeleteObject();
}

void CPIC::SetVx(int Vx)
{
	this->Vx = Vx;
}

void CPIC::SetHoriz(int Horiz)
{
	this->Horiz = Horiz;
}

void CPIC::SetVerti(int Verti)
{
	this->Verti = Verti;
}


void CPIC::InitPic(int TextVP)
{
	CClientDC dc(this);
	CRect	rc;
	GetClientRect(&rc);
	dc.SelectStockObject(NULL_BRUSH);//选取系统GDI对象（将GDI对象CBRUSH选进DC中）
	dc.SetMapMode(MM_LOMETRIC);//设置映像模式
	dc.SetViewportOrg(0,0);//设置屏幕中心为原点（窗口的原点，视口的中心）
	dc.DPtoLP(&rc);
	MaxY = rc.top-rc.bottom;
	MaxX = rc.right-rc.left;
	VertP = float(MaxY)/float(1100);
	FontSize = (int)(300/VertP);
	this->TextVP = TextVP;
	MemDC.CreateCompatibleDC(NULL);
	MemBitmap.CreateCompatibleBitmap(&dc, MaxX, MaxY);  
	CBitmap *pOldBit = MemDC.SelectObject(&MemBitmap);
	MemDC.FillSolidRect(0, 0, MaxX, MaxY, BKCLO);
}

void CPIC::ErasePic(CDC *pDC)
{
	pDC->FillSolidRect(0, 0, MaxX, MaxY, BKCLO);
}