#include <opencv_all.h>

//OpenCV_���ڻ�ϸ�˹ģ��GMM���˶�Ŀ����
int main(int argc, char** argv)
{
	VideoCapture capture;
	capture.open(0);
	BackgroundSubtractorMOG2 mog;
	Mat foreground;
	Mat background;
	Mat frame;

	if (!capture.isOpened())
	{
		cout << "read video failure" << std::endl;
		return -1;
	}
	while (1)
	{
		capture >> frame;
		flip(frame, frame, -1);
		// �˶�ǰ����⣬�����±���  
		mog(frame, foreground, 0.001);
		// ��ʴ  
		erode(foreground, foreground, cv::Mat());
		// ����  
		dilate(foreground, foreground, cv::Mat());
		mog.getBackgroundImage(background);   // ���ص�ǰ����ͼ��  
		imshow("video", foreground);
		imshow("background", background);
		waitKey(1);

		Mat elemet = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));

		/*********��ͨ�������****************/
		vector<vector<Point>> contours;
		vector<vector<Point>> filterContours;
		vector<Vec4i> hierarchy;
		hierarchy.clear();
		contours.clear();
		filterContours.clear();
		findContours(foreground, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		/*********�����************/
		vector<vector<Point>> maxContour;
		double maxArea = -1;
		int temp = -1;
		for (size_t a = 0; a < contours.size(); a++)
		{
			double area = cv::contourArea(contours[a]);
			if (area >= maxArea&&area>=100)
			{
				maxArea = area;
				temp = a;
			}
		}
	
		if (temp >= 0)
		{
			filterContours.push_back(contours[temp]);

			vector<Moments> mu(filterContours.size());
			for (int i = 0; i < filterContours.size(); i++)
			{
				mu[i] = moments(filterContours[i], false);
			}
			//��������������     
			vector<Point2f> mc(filterContours.size());
			for (int i = 0; i < filterContours.size(); i++)
			{
				mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			}
			drawContours(frame, filterContours, -1, Scalar(255, 255, 255), CV_FILLED);
			//�������������Ĳ���ʾ      
			for (int i = 0; i< filterContours.size(); i++)
			{
				circle(frame, mc[i], 2, Scalar(0, 0,255), 2, 8, 0);
			}
			
			imshow("out", frame);
			waitKey(1);
		}
		
	}
	return 0;
}