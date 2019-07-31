#include "vision/findlines.h"

FindLines::FindLines()
{
	flag_first 	= true;
	vertical    = true;
	transverse  = false;

	dis         = 0.0;
	rot         = 0.0;

	aim.setLine(Maxk, -1.0*Maxk*Image_Width/2);
}

void FindLines::findmain(cv::Mat image, int &line_threshold, cv::Mat &outimage)
{
	ans.clear();

	cv::cvtColor(image, image, CV_BGR2GRAY);
	cv::GaussianBlur(image, image, cv::Size(3,3), 0);
	cv::threshold(image, image, 1.0*line_threshold, 255.0, CV_THRESH_BINARY);

#ifdef TEST
#ifdef FIND_LINE
#ifndef TEST_ROS
	if(flag_first)
	{
		flag_first = false;
		cv::namedWindow("line_threshold", 0);
		cv::createTrackbar("x", "line_threshold", &line_threshold, 255);
		cv::namedWindow("line_canny", 0);
	}
	cv::imshow("line_threshold", image);
#endif

#ifdef TEST_ROS
	static ros::NodeHandle nh;
	static image_transport::ImageTransport it(nh);
	static image_transport::Publisher pub = it.advertise("vision/out/line_threshold", 1);
	std_msgs::Header header;
	header.seq = 0;
	header.stamp = ros::Time::now();
	header.frame_id = "line_threshold";
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
	pub.publish(msg);
#endif
#endif
#endif

	cv::Canny(image, image, 50, 100);

#ifdef TEST
#ifdef FIND_LINE
#ifndef TEST_ROS
	cv::imshow("line_canny", image);
#endif

#ifdef TEST_ROS
	static image_transport::Publisher pub2 = it.advertise("vision/out/line_canny", 1);
	header.stamp = ros::Time::now();
	header.frame_id = "line_canny";
	msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
	pub2.publish(msg);
#endif
#endif
#endif

    std::vector<cv::Vec4f> lines;
	cv::HoughLinesP(image, lines, 1, CV_PI/180, 80, 15,80);//CV_PI / 180, 100, 15, 15

	if(vertical)
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
		    	if(fabs(k) >= 1.0)
			    {
				    ans.emplace_back(LineSort(line[0],line[1]));
#ifdef TEST
				    cv::line(outimage, line[0], line[1], cv::Scalar(255, 0, 0), 6);
#endif
			    }
		    }
		    else
		    {
			    if(fabs(k) >= 1.732)
			    {
				    ans.emplace_back(LineSort(line[0],line[1]));
#ifdef TEST
				    cv::line(outimage, line[0], line[1], cv::Scalar(255, 0, 0), 6);
#endif
			    }
		    }
	    }
	}
	if(transverse)
	{
	    for(int i = 0; i<lines.size(); i++)
	    {
            cv::Point2f line[2];
            line[0].x = lines[i][0];
            line[0].y = lines[i][1];
            line[1].x = lines[i][2];
            line[1].y = lines[i][3];
		    double k = 1.0*(line[0].y - line[1].y) / (line[0].x - line[1].x);

		    if(lines.size()<4)
		    {
		        if(fabs(k) <= 1.0)
			    {
				    ans.emplace_back(LineSort(line[0],line[1]));
#ifdef TEST
					cv::line(outimage, line[0], line[1], cv::Scalar(255, 0, 0), 6);
#endif
			    }
		    }
		    else
		    {
			    if(fabs(k) <= 0.577)
			    {
				    ans.emplace_back(LineSort(line[0],line[1]));
#ifdef TEST
					cv::line(outimage, line[0], line[1], cv::Scalar(255, 0, 0), 6);
#endif
			    }
		    }
	    }
	}

	static const cv::Point ver_A(Image_Width/2, 1), ver_B(Image_Width/2, Image_Height),
	tra_A(1, Image_Height/2), tra_B(Image_Width, Image_Height/2);

#ifdef TEST
	if(vertical)
		cv::line(outimage, ver_A, ver_B, cv::Scalar(0,0,0), 6);
	if(transverse)
		cv::line(outimage, tra_A, tra_B, cv::Scalar(0,0,0), 6);
	for(unsigned int i=0; i<ans.size(); i++)
	{
		cv::line(outimage, ans[i].A, ans[i].B, cv::Scalar(0, 255, 0), 6);
	}
#endif

	sort(ans.begin(), ans.end());
	aime=finde(outimage);
	aimchange();
	outputans();

#ifdef TEST
	cv::line(outimage, aim.A, aim.B, cv::Scalar(255, 0, 255), 6);
#endif
}

LineSort FindLines::finde(cv::Mat &image)
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
	}
	else
	{
		if(vertical)
		{
			maina.x	=	Image_Width/2;
			maina.y	=	1;
			mainb.x	=	Image_Width/2;
			mainb.y	=	Image_Height;
		}
		if(transverse)
		{
			maina.x	=	1;
			maina.y	=	Image_Height/2;
			mainb.x	=	Image_Width;
			mainb.y	=	Image_Height/2;
		}
	}

#ifdef TEST
    cv::line(image, maina, mainb, cv::Scalar(0, 0, 255), 6);
#endif

	return LineSort(maina,mainb);
}

void FindLines::aimchange()
{
	cv::Point			newA, newB;
	double				eA=0.0, eB=0.0;

	if(vertical)
	{
	    eA		=	aime.A.x-aim.A.x;
	    eB		=	aime.B.x-aim.B.x;
	}
    if(transverse)
    {
        eA		=	aime.A.y-aim.A.y;
        eB		=	aime.B.y-aim.B.y;
    }

	if((fabs(eA)<1000.0||fabs(eB)<1000) && vertical)
	{
		newA.x	=	aim.A.x+static_cast<int>(KP_Of_Translation*eA);
		if(newA.x > Image_Width/2)
		{
			newA.x--;
		}
		else if(newA.x < Image_Width/2)
		{
			newA.x++;
		}
		newA.y	=	1;
		newB.x	=	aim.B.x+static_cast<int>(KP_Of_Translation*eB);
		if(newB.x > Image_Width/2)
		{
			newB.x--;
		}
		else if(newB.x < Image_Width/2)
		{
			newB.x++;
		}
		newB.y	=	Image_Height;
	}
	else if ((fabs(eA)<1000.0||fabs(eB)<1000) && transverse)
	{
        newA.y	=	aim.A.y+static_cast<int>(KP_Of_Translation*eA);
		if(newA.y > Image_Height/2)
		{
			newA.y--;
		}
		else if(newA.y < Image_Height/2)
		{
			newA.y++;
		}
        newA.x	=	1;
        newB.y	=	aim.B.y+static_cast<int>(KP_Of_Translation*eB);
		if(newB.y > Image_Height/2)
		{
			newB.y--;
		}
		else if(newB.y < Image_Height/2)
		{
			newB.y++;
		}
        newB.x	=	Image_Width;
	}
	aim	= LineSort(newA, newB);
}

void FindLines::outputans()
{
	static const cv::Point center(Image_Width/2, Image_Height/2);

	if(vertical)
	{
		if(aim.A.x==aim.B.x && aim.A.x==center.x)
			dis	= 0.0;
		else
			dis	= aim.getdistopoint(center);

		if((aim.A.x+aim.B.x)/2>center.x)
			dis	= -dis;

		if(aim.A.x==aim.B.x)
			rot	= 0.0;
		else
			rot	= atan(1.0*(aim.A.x-aim.B.x)/Image_Height);
	}

	if(transverse)
	{
		if(aim.A.y == aim.B.y && aim.A.y == center.y)
			dis	= 0.0;
		else
			dis	= aim.getdistopoint(center);

		if((aim.A.y+aim.B.y)/2>center.y)
			dis	= -dis;

		if(aim.A.y==aim.B.y)
			rot	= 0.0;
		else
			rot	= atan(1.0*(aim.A.y-aim.B.y)/Image_Width);
	}
}

void FindLines::turndir()
{
	vertical 	= !vertical;
	transverse 	= !transverse;
}

void FindLines::turnver()
{
	vertical 	= true;
	transverse 	= false;
}

void FindLines::turntra()
{
	vertical 	= false;
	transverse 	= true;
}
