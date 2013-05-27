

#include "flag_data.h"
#include "ros/ros.h"
#include <cvblob.h>
#include "../../ExternalLib/cvBlob/BlobResult.h"

#include <opencv2/legacy/blobtrack.hpp>
#include <iostream>
using namespace std;



namespace flag_space{
  
    IplImage *frame;
    sensor_msgs::CvBridge bridge;
    
    bool compare(CvPoint A,CvPoint B)
    {
        return(A.x<B.x);
        
    }
    
    
    bool compare2(CvPoint A,CvPoint B)
    {
        return(A.y<B.y);
        
    }
    
    
    
    //IplImage* FlagDetection::markFlag(const sensor_msgs::ImageConstPtr& image)
    void FlagDetection::markFlag(const sensor_msgs::ImageConstPtr& image)
    {
        try
        {
            frame = bridge.imgMsgToCv(image, "bgr8");
            cvWaitKey(10);
        }
        catch (sensor_msgs::CvBridgeException& e)
        {
            ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
        }
        
        cout<<"Flag code running"<<endl;
        cout<<"Mark 1"<<endl;
        
        
        CBlobResult blob;
        CBlob *currentBlob;
        CvRect red_rect,blue_rect;
        CvPoint *point1,*point2,boundary,most_y;
        most_y.y=0;most_y.x=0;
        
        std::vector <CvPoint> red_points,blue_points,first_red,last_blue;
        
        
        point2=new CvPoint();
        point1=new CvPoint();
        
        
        IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
        cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
        
        IplImage* imgRedBlob=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 1);
        IplImage* imgFINALBW=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,1);
        
        
        
        //First range of H for which red colour exists
        cvInRangeS(imgHSV, cvScalar(0,100,100),
                   cvScalar(5,255,255), imgRedBlob);
        
        //Second range of values for which red exists
        //        cvInRangeS(imgHSV, cvScalar(170,100,100),
        //              cvScalar(190,255,255), imgRedBlob);
        //Range of values of blue pixels
        IplImage* imgBlueBlob=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 1);
        cvInRangeS(imgHSV, cvScalar(100,110,110), cvScalar(130,255,255), imgBlueBlob);
        
        //creating a final image
        
        IplImage* imgFinal=  cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
        
        //identifying the red blobs
        blob=CBlobResult(imgRedBlob,NULL,0);
        
        //counter of the red blobs in a specific range of size
        int valid_blob_counter_red=0;
        
        for(int i=0;i < blob.GetNumBlobs();i++)
        {
            currentBlob=blob.GetBlob(i);
            
            red_rect=currentBlob->GetBoundingBox();
            
            
            //only the  red blobs larger than 3x3 pixel
            if(red_rect.x*red_rect.y>(100))
            {
                // cout<<"mark"<<endl;
                
                if(++valid_blob_counter_red==1)
                {
                    point2->x=(red_rect.x);
                    point2->y=red_rect.y;
                    
                    red_points.push_back (*point2);
                }
                else
                {
                    (*point1)=(*point2);
                    
                    
                    //cout<<"redx="<<point1->x<<"\tredy="<<point1->y<<"\n";
                    point2->x=red_rect.x+red_rect.width/2;
                    point2->y=red_rect.y+red_rect.height/2;
                    //points at the center of the rectangle are stored in the memmory
                    
                    //storing the red pixels
                    red_points.push_back ((*point2));
                    
                }
                
            }
        }
        
        cout<<"Mark 2"<<endl;
        
        //sorting the red pixels according to the x cordinates
        std::sort(red_points.begin(),red_points.end(),compare);
        
        
        IplImage* imgFinal2=  cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
        
        //blue blobs are identified
        blob=CBlobResult(imgBlueBlob,NULL,0);
        
        //counting the blue blobs
        int valid_blob_counter_blue=0;
        
        for(int i=0;i < blob.GetNumBlobs();i++)
        {
            
            
            
            currentBlob=blob.GetBlob(i);
            blue_rect=currentBlob->GetBoundingBox();
            
            
            //size more than 3x3 are valid
            if(blue_rect.height*blue_rect.width>100)
                
            {
                if(++valid_blob_counter_blue==1)
                {
                    
                    //storing the pixels
                    point2->x=blue_rect.x+blue_rect.width;
                    point2->y=blue_rect.y+blue_rect.height;
                    
                    blue_points.push_back (*point2);
                }
                
                else
                {
                    
                    (*point1)=(*point2);
                    //cout<<"bluex="<<point1->x<<"\tbluey="<<point1->y<<"\n";
                    
                    //cout<<"width:"<<blue_rect.width<<"\tHieght:"<<blue_rect.height<<endl;
                    point2->x=blue_rect.x+blue_rect.width/2;
                    point2->y=blue_rect.y+blue_rect.height/2;
                    
                    //storing the blue pixels in a vector
                    blue_points.push_back(*point2);
                    
                }
                
                
            }
            
        }
        cout<<"Mark 3"<<endl;
        //sorting according to the x coordinates
        std::sort(blue_points.begin(),blue_points.end(),compare);
        cout<<"Mark 4"<<endl;
        
        
        
        if(valid_blob_counter_blue==0||valid_blob_counter_red==0)
        {
            cout<<"red or blue or both flags not detected"<<endl;
            cvShowImage("Final B/W Image",imgFINALBW);
            cvReleaseImage(&imgFinal2);
            cvReleaseImage(&imgFinal);
            cvReleaseImage(&imgHSV);
            //cvReleaseImage(&frame);
            cvReleaseImage(&imgRedBlob);
            cvReleaseImage(&imgBlueBlob);
            cvReleaseImage(&imgFINALBW);
            return;
        }
        
        cout<<"Mark 5"<<endl;
        //Printing the values of red and blue points
        
        /*for(int i=0;i<red_points.size();i++)
         cout<<"red points are ("<<red_points[i].x<<","<<red_points[i].y<<")"<<endl;
         
         for(int i=0;i<blue_points.size();i++)
         cout<<"blue points are ("<<blue_points[i].x<<","<<blue_points[i].y<<")"<<endl;
         
         cout<<"img width="<<imgFinal->width<<"\timg height"<<imgFinal->height<<endl;
         */
        //DRAWING The RED LINES
        int xpos;
        
        int linenumber_red=1;
        
        int flag2=1;
        
        int i=0;
        int flag=1;
        
        if(valid_blob_counter_red==0)
        {
            
            point2->x=0;
            point2->y=imgFinal->height;
            
            red_points.push_back(*point2);
            
            first_red.push_back(*point2);
        }
        
        cout<<"Mark 6"<<endl;
        
        if(valid_blob_counter_blue==0)
        {
            point2->x=imgFinal->width;
            point2->y=imgFinal->height;
            
            blue_points.push_back(*point2);
            last_blue.push_back(*point2);
            
        }
        
        
        cout<<"Mark 7"<<endl;
        //Iterating through all the blobs and seperating them into different lines
        for(i=0 ,xpos=0,flag=1;i<red_points.size();i++)
        {
            
            if(flag)
            {
                
                xpos=red_points[i].x;
                flag=0;
                (*point2)=red_points[i];
                //also storing the point of each lne with y coordinate closer to the bottom of the image
                if(most_y.y<red_points[i].y)
                {
                    most_y=red_points[i];
                    
                }
            }
            
            
            //If the x position of the two blobs is greater than a certain
            //threshold then it must be a different line
            else if((red_points[i].x-xpos)>MAX_HOR_DIST)
            {
                
                flag=1;
                flag2=0;
                i--;
                boundary.x=most_y.x;
                boundary.y=imgFinal->height;
                cvLine(imgFinal, most_y,boundary,
                       cvScalar(0, 0, 255, 0 ),
                       8,8,0 );
                if(linenumber_red==1)
                {
                    //storing the first red line as there may
                    //be a possibility that
                    //the vehicle can
                    //go left of the red line on the extreme left
                    
                    
                    if(i==0)
                    {
                        
                        first_red.push_back(red_points[0]);
                    }
                    
                    
                    else
                    {
                        
                        first_red.assign(red_points.begin(),red_points.begin()+i);
                    }
                    
                    
                }
                
                
                linenumber_red++;
                continue;
                
            }
            
            
            else
            {
                //drawing lines
                *point1=*point2;
                *point2=red_points[i];
                cvLine(imgFinal, *point1,*point2,
                       cvScalar(0, 0, 255, 0 ),
                       8,8,0 );
                
                if(most_y.y<red_points[i].y)
                {
                    
                    most_y=red_points[i];
                    //cout<<"Replacing the max value of y"<<endl;
                    
                }
            }
            
        }
        
        cout<<"Mark 8"<<endl;
        cout<<"Mark 9"<<endl;
        
        if(flag2)
        {
            
            
            {
                
                first_red.assign(red_points.begin(),red_points.begin()+i);
            }
        }
        cout<<"Mark 10"<<endl;
        
        boundary.x=most_y.x;
        boundary.y=imgFinal->height;
        cvLine(imgFinal, most_y,boundary,
               cvScalar(0, 0, 255, 0 ),
               8,8,0 );
        
        
        most_y.x=0;
        most_y.y=0;
        
        
        int LineStartsFromi;
        
        //DRAWING THE BLUE LINES
        cout<<"Mark 11"<<endl;
        for(int i=0, xpos=0,flag=1;i<blue_points.size();i++)
        {
            
            if(flag)
            {
                LineStartsFromi=i;
                //we need the first coordinate of the last blue line
                
                xpos=blue_points[i].x;
                flag=0;
                *point2=blue_points[i];
                
                if(most_y.y<blue_points[i].y)
                {
                    most_y=blue_points[i];
                    
                }
            }
            
            
            else if((blue_points[i].x-xpos)>MAX_HOR_DIST)
                //differentiating between the lines with the distance of x coordinates
            {
                
                flag=1;
                i--;
                boundary.x=most_y.x;
                boundary.y=imgFinal->height;
                cvLine(imgFinal, most_y,boundary,
                       cvScalar(255, 0, 0, 0 ),
                       8,8,0 );
                
                continue;
            }
            
            else
            {
                *point1=*point2;
                *point2=blue_points[i];
                
                if(most_y.y<blue_points[i].y)
                {
                    most_y=blue_points[i];
                    
                }
                cvLine(imgFinal, *point1,*point2,
                       cvScalar(255, 0, 0, 0 ),
                       8,8,0 );
                
            }
            
            
            
        }
        
        cout<<"Mark 12"<<endl;
        last_blue.assign(blue_points.begin()+LineStartsFromi,blue_points.end());
        
        boundary.x=most_y.x;
        boundary.y=imgFinal->height;
        cvLine(imgFinal, most_y,boundary,
               cvScalar(255, 0, 0, 0 ),
               8,8,0 );
        
        
        cout<<"Mark 13"<<endl;
        
        //sorting the first red and last blue line according to the y cordinate
        
        sort(first_red.begin(),first_red.end(),compare2);
        sort(last_blue.begin(),last_blue.end(),compare2);
        
        
        
        //the below may be used to print the values of the red and blue pixels
        
        /*
         std::vector<CvPoint>::iterator it;
         
         std::cout << "first_red contains:";
         for (it=first_red.begin(); it<first_red.end(); it++)
         
         {    std::cout <<"("<< it->x<<" "<<it->y<<")"<<endl;}
         
         cout<<endl;
         
         std::cout << "last_blue contains:";
         for (it=last_blue.begin(); it<last_blue.end(); it++)
         {std::cout <<"("<< it->x<<" "<<it->y<<")"<<endl;}
         */
        
        
        //To compare which side of the lanes we should pass through
        
        // cout<<"red begins from"<<red_points.begin()->x<<"\tblue begins from"<<blue_points.begin()->y<<endl;
        
        //cout<<"red ends with"<<(red_points.end()-1)->x<<"\tblue ends with"<<(blue_points.end()-1)->y<<endl;
        
        
        if((red_points[0].x)<(blue_points[0].x))
        {
            
            printf("First red on the left of first blue\n");
            
            //make the region on the left DRIVABLE
            
            
            
            
            for(int i=(first_red[0].y+10); i<(imgFinal->height);i++)
            {
                
                
                for(int j=0;j<(imgFinal->width);j++)
                {
                    
                    if(IMGDATA(imgFinal,i,j,2)==255)
                    {
                        
                        point1->x=0;
                        point1->y=i;
                        point2->x=j;
                        point2->y=i;
                        //marking thr drivable region green
                        cvLine(imgFinal2,*point1,*point2,cvScalar(0,255,0,0),1,8,0);
                        break;
                    }
                }
            }
            
            
            
        }
        
        cout<<"Mark 14"<<endl;
        if(((blue_points.end()-1)->x)>((red_points.end()-1)->x))
        {
            printf("Last blue on the right\n");
            //make the region on the right DRIVABLE
            for(int i=last_blue[0].y;i<imgFinal->height;i++)
            {
                for(int j=imgFinal->width;j>0;j--)
                {
                    if(IMGDATA(imgFinal,i,j,0)==255)
                    {
                        point1->x=j;
                        point1->y=i;
                        point2->x=imgFinal->width;
                        point2->y=i;
                        cvLine(imgFinal2,*point1,*point2,cvScalar(0,255,0,0),1,8,0);
                        break;
                    }
                }
            }
            
            
        }
        
        cout<<"Mark 14"<<endl;
        
        for(int i=0;i<imgFinal->height;i++)
        {
            flag=0;
            //flag is kept in order to take care of
            //cases wth multiple red and blue combinations
            for(int j=0;j<imgFinal->width;j++)
            {
                if((int)IMGDATA(imgFinal,i,j,0)==255)
                {
                    flag=1;
                    point1->x=j;
                    point1->y=i;
                    
                }
                
                else if( ( (int)IMGDATA(imgFinal,i,j,2)==255)&&(flag))
                {
                    flag=0;
                    
                    point2->x=j;
                    point2->y=i;
                    cvLine(imgFinal2,*point1,*point2,cvScalar(0,255,0,0),1,8,0);
                    
                }
                
            }
            
        }
        
        cout<<"Mark 15"<<endl;
        
        
        
        cvInRangeS(imgFinal2, cvScalar(0,250,0),
                   cvScalar(0,255,0), imgFINALBW);
        
        
        
        
        cv::waitKey(10);
        
        free(point1);
        free(point2);
        cout<<"Mark 16"<<endl;
        cvShowImage("Video", frame);
        cvShowImage("imgBlue",imgBlueBlob);
        cvShowImage("imgRed",imgRedBlob);
        cvShowImage("Final image",imgFinal);
        //cvShowImage("Final image2",imgFinal2);
        cvShowImage("Final B/W Image",imgFINALBW);
        
        //Clean up used images
        
        
        
        cout<<"Mark 17"<<endl;
        
        
        //ros::init(argc, argv, "flag_pub");
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("flag/image", 10);
        
        sensor_msgs::ImagePtr msg;
        
        
        IplImage* imgPublisher;
        
        
        
        ros::Rate loop_rate(10);
        int counter=1;
        while (nh.ok())
        {
            imgPublisher =imgFINALBW;
            if(!imgPublisher)
            {
                throw "Unable to get imgPublisher";
                break;
            }
            ROS_INFO("%d", counter);
            
            counter++;
            if (counter > 5)
                counter = 1;
            msg = sensor_msgs::CvBridge::cvToImgMsg(imgFINALBW, "bgr_Flag");
            pub.publish(msg);
            //ros::spinOnce();
            loop_rate.sleep();
        }
        //cvReleaseImage(&frame);
        
        cout<<"Mark 18"<<endl;
        
        
        cvReleaseImage(&imgFinal2);
        cvReleaseImage(&imgFinal);
        cvReleaseImage(&imgHSV);
        //cvReleaseImage(&frame);
        cvReleaseImage(&imgRedBlob);
        cvReleaseImage(&imgBlueBlob);
        cvReleaseImage(&imgFINALBW);
        
        return;
        // return imgFINALBW;
        
        
        
    }
    
    
}
