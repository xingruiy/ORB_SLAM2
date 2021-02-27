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
#include <xyutils/Camera/InputFactory.h>
#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./custom_input path_to_vocabulary path_to_settings url" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    auto camera = InputFactory::GetInputMethod(argv[3]);

    // Main loop
    size_t id;
    double time;
    cv::Mat imRGB, imD;

    while (camera->HasImages())
    {
        if (camera->GetNext(id, imRGB, imD, time))
        {
            // Pass the image to the SLAM system
            SLAM.TrackRGBD(imRGB, imD, time);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    std::stringstream ss1, ss2;
    ss1 << camera->GetUniqueName() << ".txt";
    ss2 << camera->GetUniqueName() << "_keyframe.txt";

    SLAM.SaveTrajectoryTUM(ss1.str().c_str());
    SLAM.SaveKeyFrameTrajectoryTUM(ss2.str().c_str());

    return 0;
}
