/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
 
#include <opencv2/core/core.hpp>
 
#include <System.h>

using namespace std;

bool isInteger(const std::string &s)
{
  return (s.size() == 1 && std::isdigit(s[0]));
}


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: " << argv[0] << " path_to_vocabulary path_to_settings path_to_video_or_cam_id" << endl;
        return 1;
    }

   
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.reserve(2000);

    cv::VideoCapture capture;
    const std::string filename{argv[3]};
    if(isInteger(filename))
    {
      capture.open(std::atoi(filename.c_str()));
    }
    else
    {
      capture.open(filename);
    }
    // capture.open( std::string( argv[3] ) );
    // capture.open( 0 );
    if( !capture.isOpened( ) )
    {
        cerr << "Could not open video file " << argv[3] << endl;
        return EXIT_FAILURE;
    }

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;
    double tframe{0};
    while(true)
    {
        // Read image from file
        capture >> im;

        if(im.empty())
        {
            break;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        tframe += .05;

   }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(const auto& t : vTimesTrack)
    {
        totaltime+=t;
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

