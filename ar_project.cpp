/*
 * *************************************************************************
 * Augmented reality - final project
 * Deepak R Karishetti
 * 10846936
 *  ***************************************************************************
 */

#include <opencv2/opencv.hpp>
#include <iostream>

// GLOBAL params
cv::Mat frame;

// RGB <--> YCrCb 
cv::Mat YCbCr_image;
cv::Mat fin_det_image;
cv::Mat roi_image;

// Y -> [16,235] 
// Cr & Cb -> [16,240] 
// Cr -> [130,180]
// Cb -> [80,150]

int Cr_l = 16;
const int Cr_u = 240;
int Cb_l = 16;
const int Cb_u = 240;

std::vector<std::vector<cv::Point> > stroke(1); // source: https://processing.org/reference/stroke_.html
int stroke_index = 0;
bool draw_mode = false; //init

//**************************************************************************
// compare each pixel values with the bounds set to detect the hand region
void hand_detect(cv::Mat YCbCr_image, cv::Mat fin_det_image)
{
  int Cr_val;
  int Cb_val;
  for (int i=0; i<YCbCr_image.rows; i++)
  {
    for (int j=0; j<YCbCr_image.cols; j++)
    {
      Cr_val = YCbCr_image.at<cv::Vec3b>(i,j)[1];
      Cb_val = YCbCr_image.at<cv::Vec3b>(i,j)[2];
      
      if (Cr_val>Cr_l && Cr_val<Cr_u && Cb_val>Cb_l && Cb_val<Cb_u)
      {
        fin_det_image.at<uchar>(i,j) = 255;
      }
      else
      {
        fin_det_image.at<uchar>(i,j) = 0;
      }
    }
  }
}

//**************************************************************************
// find distance between two points
int distance(cv::Point* pt_1, cv::Point* pt_2)
{
  return (((*pt_1).x-(*pt_2).x)*((*pt_1).x-(*pt_2).x) + ((*pt_1).y-(*pt_2).y)*((*pt_1).y-(*pt_2).y));
}
  
//**************************************************************************
// detect the gesture based on the fingers
int gesture_detect(cv::Mat frame)
{
  fin_det_image = cv::Mat::zeros(frame.size(),CV_8U);
  // int fr_s = frame.cols/2;
  int fr_s = 200;
  cv::Rect roi(frame.cols/2-fr_s, 0, frame.cols/2+fr_s, frame.rows);
  roi_image = frame(roi);
  cv::cvtColor(roi_image, YCbCr_image, CV_BGR2YCrCb);
  hand_detect(YCbCr_image, fin_det_image);
  
  cv::namedWindow("Detect region", cv::WINDOW_AUTOSIZE);

  // trackbar to set the Cr Cb  values 
  cv::createTrackbar("Cb lower bound", "Detect region", &Cb_l, Cb_u);
  cv::createTrackbar("Cr lower bound", "Detect region", &Cr_l, Cr_u);
  cv::createTrackbar("Cb upper bound", "Detect region", &Cb_l, Cb_u);
  cv::createTrackbar("Cr upper bound", "Detect region", &Cr_l, Cr_u);
  cv::imshow("Detect region", fin_det_image);
  
  // find contours
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  
  cv::findContours(fin_det_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
  
  // draw contours 
  cv::Mat drawing = cv::Mat::zeros(fin_det_image.size(), CV_8UC3);
  int max_s = 0;
  int n_max;
  int cur_size;

  for (int i=0; i<contours.size(); i++)
  {
    cur_size = cv::contourArea(contours[i]);

    if (cur_size > max_s)
    {
      max_s = cur_size;
      n_max = i;
    }
  }

  if (max_s <= 10000)
  {
    return 0;
  }

  // hull results
  std::vector<std::vector<cv::Point> > hull(1);
  cv::convexHull(cv::Mat(contours[n_max]), hull[0], false);

  std::vector<std::vector<int> > hull_2(1);
  cv::convexHull(cv::Mat(contours[n_max]), hull_2[0], false);

  // using the convexity defects from the hull 
  std::vector<cv::Vec4i> defects;
  cv::convexityDefects(cv::Mat(contours[n_max]), hull_2[0], defects);

  // approx 
  int no_def = 0;
  int def_avg = 0;
  int defects_max_h = frame.rows;
  int n_higher = 0;

  for (int i=0; i<defects.size(); i++)
  {
    if (defects[i][3] > 4000)
    {
      cv::circle(drawing, cv::Point(contours[n_max][defects[i][2]].x+frame.cols/2-fr_s, contours[n_max][defects[i][2]].y), 3, cv::Scalar(0,0,255), -1);
      def_avg += contours[n_max][defects[i][2]].y;
      no_def++;

      if (contours[n_max][defects[i][2]].y < defects_max_h)
      {
        defects_max_h = contours[n_max][defects[i][2]].y;
      }
    }
  }

  if (no_def!=0)
  {
    def_avg = def_avg/no_def;
  }

  std::vector<std::vector<cv::Point> > hand(1);
  cv::approxPolyDP(hull[0], hand[0], 10, false);

  for (int i=0; i<hand[0].size()-1; )
  {
    if (distance(&hand[0][i], &hand[0][i+1]) < 800)
    {
      hand[0][i].x = (hand[0][i].x+hand[0][i+1].x)*0.5;
      hand[0][i].y = (hand[0][i].y+hand[0][i+1].y)*0.5;

      hand[0].erase(hand[0].begin()+i+1);
    }
    else
    {
      i++;
    }
  }

  if (distance(&hand[0][0], &hand[0].back()) < 800)
  {
    hand[0].back().x = (hand[0][0].x + hand[0].back().x)*0.5;
    hand[0].back().y = (hand[0][0].y + hand[0].back().y)*0.5;

    hand[0].erase(hand[0].begin());
  }

  int top_y = frame.rows;
  int top_index = 0;
  for (int i=0; i<hand[0].size(); i++)
  {
    hand[0][i].x += frame.cols/2 - fr_s;
    cv::circle(drawing, hand[0][i], 3, cv::Scalar(200, 255, 200), -1);

    if (hand[0][i].y <= defects_max_h)
    {
      n_higher++;
    }
    if (hand[0][i].y < top_y)
    {
      top_index = i;
      top_y = hand[0][i].y;
    }
  }

  for (int i=0; i<contours[n_max].size(); i++)
  {
    contours[n_max][i].x += frame.cols/2 - fr_s;
  }

  cv::drawContours(drawing, contours, n_max, cv::Scalar(0, 0, 255), 2, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

  cv::drawContours(drawing, hand, 0, cv::Scalar(0, 255, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

  // std::cout << "where?" << "\n";

  cv::namedWindow("Convex hull of hand", cv::WINDOW_AUTOSIZE);
  cv::imshow("Convex hull of hand", drawing);

  if (no_def >= 5)
  {
    for (int i=0; i<hand[0].size(); i++)
    {
      if (hand[0][i].y < def_avg)
      {
        cv::circle(frame, hand[0][i], 6, cv::Scalar(0, 0, 255), -1);
      }
    }

    if (draw_mode)
    {
      std::vector<cv::Point> temp_p;
      stroke.push_back(temp_p);
      stroke_index++;
    }

    draw_mode = false;
    
    return 1;
  }

  else if ((n_higher >=3) && (no_def <=2) && (hand[0].size() >= 5))
  {
    for (int i=0; i<stroke.size(); i++)
    {
      stroke[i].clear();
    }
    stroke.clear();

    std::vector<cv::Point> temp_p;
    stroke.push_back(temp_p);
    stroke_index = 0;

    draw_mode = false;

    return 3;
  }

  else
  {
    cv::circle(frame, hand[0][top_index], 8, cv::Scalar(20, 200, 200), -1);
    draw_mode = true;

    if (draw_mode == true)
    {
      stroke[stroke_index].push_back(cv::Point(hand[0][top_index].x, hand[0][top_index].y));
    }

    return 2;
  }
}


//**************************************************************************
// draw 
void draw(cv::Mat frame)
{
  for (int i=0; i<stroke.size(); i++)
  {
    if (stroke[i].size() == 0)
      continue;
      for (int j=0; j<stroke[i].size()-1; j++)
      {
        cv::line(frame, stroke[i][j], stroke[i][j+1], cv::Scalar(255, 0, 0), 4);
      }
    }
}

//**************************************************************************
int main()
{
  int source = 0;
  cv::VideoCapture video(source);
  // check for source
  if (!video.isOpened())
  {
    std::cout << "Error opening the video source!" << "\n";
    system("PAUSE");
    return -1;
  }
  
  // display windows
  cv::namedWindow("Source", cv::WINDOW_AUTOSIZE);

  // mode variable determining which function to perform
  int mode_val;

  // write video to file
  int w = video.get(cv::CAP_PROP_FRAME_WIDTH);
  int h = video.get(cv::CAP_PROP_FRAME_HEIGHT);
  const cv::String video_out("AR_project_output.avi");
  cv::VideoWriter output_video(video_out, cv::VideoWriter::fourcc('D','I','V','X'), 30.0, cv::Size(int(w), int(h)), true);


  while (true)
  {
    video >> frame;

    // if no more frames
    if (frame.empty())
    {
      std::cout << "Error: no more frames available!" << "\n";
      break;
    }

    cv::flip(frame,frame,1);

    mode_val = gesture_detect(frame);
    
    cv::putText(frame,"MODE: ",cv::Point(10,50), cv::FONT_HERSHEY_TRIPLEX,1.0,CV_RGB(0,255,0),2);

    if (mode_val == 1)
    {
      cv::putText(frame,"Hold",cv::Point(120,50), cv::FONT_HERSHEY_DUPLEX,1.0,
CV_RGB(255,0,0),2);
    }

    else if (mode_val == 2)
    {
      cv::putText(frame,"Draw",cv::Point(120,50), cv::FONT_HERSHEY_DUPLEX,1.0,
CV_RGB(0,255,255),2);
    }

    else if (mode_val == 3)
    {
      cv::putText(frame,"Clear all",cv::Point(120,50), cv::FONT_HERSHEY_DUPLEX,1.0,
CV_RGB(0, 0, 0),2);
    }

    draw(frame);

    // write to output video file
    output_video << frame;

    // show
    cv::imshow("Source",frame);

    // exit on key press
    if (cv::waitKey(33) == 27)
    {
      break;
    }
  }

  return EXIT_SUCCESS;
}
