#include "vision/findlines.h"

FindLines::FindLines()
{
	ans.clear();

	nFrmNum	= 0;
	heng    = false;
	shu     = true;
}

void FindLines::findmain(cv::Mat tpf, int &line_threshold, cv::Mat &outimage)
{
	pFrame = tpf;
	ans.clear();
	pFrame = pFrame(cv::Rect(0, CutHeight, pFrame.cols, pFrame.rows - CutHeight));
	nFrmNum++;
	if(nFrmNum == 1)
	{
		pCutFrame = pFrame.clone();
		cv::cvtColor(pCutFrame, pCutFrImg, CV_BGR2GRAY);
		initaim();
	}
	else
	{
        pCutFrame = pFrame.clone();
        cv::cvtColor(pCutFrame, pCutFrImg, CV_BGR2GRAY);
        cv::GaussianBlur(pCutFrImg, pCutFrImg, cv::Size(3,3), 0);
		cv::threshold(pCutFrImg, pCutFrImg, line_threshold, 255.0, CV_THRESH_BINARY);

#ifdef TEST
		static bool flag_first = true;
		if(flag_first)
		{
			flag_first = false;
			cv::namedWindow("line_threshold", 0);
			cv::createTrackbar("x", "line_threshold", &line_threshold, 255);
			cv::namedWindow("line_canny", 0);
		}
		cv::imshow("line_threshold", pCutFrImg);
#endif
		//进行形态学滤波，去掉噪音
		//cvErode(pCutFrImg, pCutFrImg, 0, 3);
		//cvDilate(pCutFrImg, pCutFrImg, 0, 3);

		//canny变化
		cv::Canny(pCutFrImg, pCutFrImg, 50, 100);
#ifdef TEST
		cv::imshow("line_canny", pCutFrImg);
#endif
	}
	cv::HoughLinesP(pCutFrImg, lines, 1, CV_PI / 180, 80, 15,80);//CV_PI / 180, 100, 15, 15
	litknum=0;
	if(shu)
	{
	    for(int i = 0; i<lines.size(); i++)
	    {
		    cv::Point2f line[2];
		    line[0].x = lines[i][0];
            line[0].y = lines[i][1];
            line[1].x = lines[i][2];
            line[1].y = lines[i][3];
		    double k = ((line[0].y - line[1].y)*1.0 / (line[0].x - line[1].x));

		    if(lines.size()<4)
		    {
		    	if(fabs(k)>=1)
			    {
				    ans.emplace_back(Linezxz(line[0],line[1]));
				    cv::line(outimage, line[0], line[1], CV_RGB(0, 0, 255), 6, CV_AA);
			    }
		    }
		    else
		    {
			    if(fabs(k)>=1)
			    {
				    ans.emplace_back(Linezxz(line[0],line[1]));
				    cv::line(outimage, line[0], line[1], CV_RGB(0, 0, 255), 6, CV_AA);
			    }
			    else
			    {
				    litknum++;
			    }
		    }
	    }
	}
	if(heng)
	{
	    for(int i = 0; i<lines.size(); i++)
	    {
            cv::Point2f line[2];
            line[0].x = lines[i][0];
            line[0].y = lines[i][1];
            line[1].x = lines[i][2];
            line[1].y = lines[i][3];
		    double k = ((line[0].y - line[1].y)*1.0 / (line[0].x - line[1].x));

		    if(lines.size()<4)
		    {
		        if((fabs(k) < 0.5))
			    {
				    ans.emplace_back(Linezxz(line[0],line[1]));
					cv::line(outimage, line[0], line[1], CV_RGB(0, 0, 255), 6, CV_AA);
			    }
		    }
		    else
		    {
			    if((fabs(k) < 0.4))
			    {
				    ans.emplace_back(Linezxz(line[0],line[1]));
					cv::line(outimage, line[0], line[1], CV_RGB(0, 0, 255), 6, CV_AA);
			    }
			    else
			    {
				    litknum++;
			    }
		    }
	    }
	}

	cv::Point aaa,bbb,ccc,ddd;
	aaa.x = 320;
	aaa.y = 1;
	bbb.x = 320;
	bbb.y = 480-CutHeight;
	ccc.x = 1;
	ccc.y = 240;
	ddd.x = 640;
	ddd.y = (480-CutHeight) / 2;

#ifdef TEST
	if(shu)
		cv::line(outimage, aaa, bbb, CV_RGB(0,0,0), 6, CV_AA);
	if(heng)
		cv::line(outimage, ccc, ddd, CV_RGB(0,0,0), 6, CV_AA);
	for(unsigned int i=0; i<ans.size(); i++)
	{
		cv::line(outimage, ans[i].A, ans[i].B, CV_RGB(255, 0, 0), 6, CV_AA);
	}
#endif

	sort(ans.begin(), ans.end());
	aime=finde(outimage);
	aimchange();
	outputans();

#ifdef TEST
	cv::line(outimage, aim.A, aim.B, CV_RGB(0, 0, 255), 6, CV_AA);
#endif
}

void FindLines::initaim()
{
	aim.setLine(Maxk, -Maxk*320.0);
}

Linezxz FindLines::finde(cv::Mat &image)
{
	unsigned long midnum = 0;
	cv::Point maina, mainb;
	if(!ans.empty())
	{
		if(ans.size()>=3)
		{
			if(ans.size()%2==0)
			{
				midnum=ans.size()/2-1;
				maina.x=(ans[midnum].A.x+ans[midnum+1].A.x)/2;
				maina.y=(ans[midnum].A.y+ans[midnum+1].A.y)/2;
				mainb.x=(ans[midnum].B.x+ans[midnum+1].B.x)/2;
				mainb.y=(ans[midnum].B.y+ans[midnum+1].B.y)/2;
			}
			else
			{
				midnum=ans.size()/2;
				maina.x=(ans[midnum-1].A.x+ans[midnum].A.x+ans[midnum+1].A.x)/3;
				maina.y=(ans[midnum-1].A.y+ans[midnum].A.y+ans[midnum+1].A.y)/3;
				mainb.x=(ans[midnum-1].B.x+ans[midnum].B.x+ans[midnum+1].B.x)/3;
				mainb.y=(ans[midnum-1].B.y+ans[midnum].B.y+ans[midnum+1].B.y)/3;
			}
		}
		else if(ans.size()==2)
		{
			midnum=0;
			maina.x=(ans[midnum].A.x+ans[midnum+1].A.x)/2;
			maina.y=(ans[midnum].A.y+ans[midnum+1].A.y)/2;
			mainb.x=(ans[midnum].B.x+ans[midnum+1].B.x)/2;
			mainb.y=(ans[midnum].B.y+ans[midnum+1].B.y)/2;
		}
		else
		{
			maina=ans[0].A;
			mainb=ans[0].B;
		}

#ifdef TEST
		cv::line(image, maina, mainb, CV_RGB(0, 255, 0), 6, CV_AA);
#endif
	}
	else
	{
		if(shu)
		{
			maina.x	=	320;
			maina.y	=	1;
			mainb.x	=	320;
			mainb.y	=	480-CutHeight;
		}
		if(heng)
		{
			maina.x	=	1;
			maina.y	=	240;
			mainb.x	=	640 -CutHeight;
			mainb.y	=	240;
		}
	}
	return Linezxz(maina,mainb); 
}

void FindLines::aimchange()
{
	static double		eAI[3]={0};
	static double		eBI[3]={0};
	cv::Point			newA, newB;
	double				eA=0.0, eB=0.0;

	if(shu)
	{
	    eA		=	aime.A.x-aim.A.x;
	    eB		=	aime.B.x-aim.B.x;
	}
    if(heng)
    {
        eA		=	aime.A.y-aim.A.y;
        eB		=	aime.B.y-aim.B.y;
    }

	if((fabs(eA)<1000.0||fabs(eB)<1000) && shu)
	{
		newA.x	=	aim.A.x+static_cast<int>(KPOfTranslation*eA);
		newA.y	=	1;
		newB.x	=	aim.B.x+static_cast<int>(KPOfTranslation*eB);
		newB.y	=	480-CutHeight;
		aim		=	Linezxz(newA, newB);
	}
	else if ((fabs(eA)<1000.0||fabs(eB)<1000) && heng)
	{
        newA.y	=	aim.A.y+static_cast<int>(KPOfTranslation*eA);
        newA.x	=	1;
        newB.y	=	aim.B.y+static_cast<int>(KPOfTranslation*eB);
        newB.x	=	640-CutHeight;
        aim		=	Linezxz(newA, newB);
	}
}

void FindLines::outputans()
{
	static const int cenx = 320;
	static const int ceny = (320-CutHeight)/2;
	static const int tianx = 320;
	static const int tiany = 500 / 2;

	cv::Point cen, tian;
	cen.x =	cenx;
	cen.y =	ceny;
	tian.x = tianx;
	tian.y = tiany;

	if(shu)
	{
		if(aim.A.x==aim.B.x && aim.A.x==cenx)
			dis	= 0.0;
		else
			dis	= aim.getdistopoint(cen);

		if((aim.A.x+aim.B.x)/2>cenx)
			dis	= -dis;

		if(aim.A.x==aim.B.x)
			rot	= 0.0;
		else
			rot	= atan( static_cast<double>(aim.A.x-aim.B.x)/(480.0-CutHeight) );
	}

	if(heng)
	{
		if(aim.A.y == aim.B.y && aim.A.y == tiany)
			dis	= 0.0;
		else
			dis	= aim.getdistopoint(tian);

		if((aim.A.y+aim.B.y)/2>tiany)
			dis	= -dis;

		if(aim.A.y==aim.B.y)
			rot	= 0.0;
		else
			rot	= atan( static_cast<double>(aim.A.x-aim.B.x)/480.0 );
	}
}
