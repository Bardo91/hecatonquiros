#pragma once

typedef CList<short, short&>  CPntLst;

#define		VMINX		3

#define		BLACKCLO	RGB(0,0,0)
#define		REDCLO		RGB(255,0,0)
#define		GREENCLO	RGB(0,255,0)
#define		BLUECLO		RGB(0,0,255)
#define		BKCLO		RGB(255,255,255)
#define		YELLOWCLO	RGB(255,255,0)

#define		BLACKSEL	(1<<0)
#define		REDSEL		(1<<1)
#define		GREENSEL	(1<<2)
#define		BLUESEL		(1<<3)
#define		YELLOWSEL	(1<<4)

// CPIC 对话框
class CPIC : public CWnd
{
public:
	CPIC();   // 标准构造函数
	virtual ~CPIC();

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	DECLARE_MESSAGE_MAP()
	afx_msg void OnPaint();
public:
	int		LineSel;
	int		TextVP;
public:
	void	InitPic(int TextVP);
	void	AddBlackLineDat(short y);
	void	AddRedLineDat(short y);
	void	AddGreenLineDat(short y);
	void	AddBlueLineDat(short y);
	void	AddYellowLineDat(short y);
	void	SetVx(int Vx);
	void	SetHoriz(int Horiz);
	void	SetVerti(int Verti);
private:
	CDC		MemDC; 
	CBitmap MemBitmap;
	CPntLst	BlackLine;
	CPntLst	RedLine;
	CPntLst	GreenLine;
	CPntLst	BlueLine;
	CPntLst	YellowLine;
	int		MaxY;
	int		MaxX;
	int		Vx;
	int		Horiz;
	int		Verti;
	float	VertP;
	int		FontSize;
private:
	void	DrawScale(CDC *pDC, COLORREF rgb);
	void	DrawPIC(CDC *pDC);
	void	DrawLine(CPntLst &LineList, CDC *pDC, COLORREF rgb);
	void	AddLineDat(CPntLst &LineList, short y);
	void	ErasePic(CDC *pDC);
};
