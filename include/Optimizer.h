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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

/**
 * 非线性优化类
 */
class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);

    /**
     * @brief 全局BA
     * @param pMap:地图
     * @param nIterations:迭代次数
     * @param pbStopFlag:是否停止FLAG
     * @param nLoopKF:回环帧ID
     * @param bRobust:是否设置鲁棒核
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);

    /**
     * @brief 这里是进行局部的BA优化
     * @param pKF : 输入关键帧
     * @param pbStopFlag : 输入是否停止优化flag
     * @param pMap : 输入地图点
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);

    /**
     * @brief 这里的顶点只有一个,那就是相机的Pose,因此这里只优化当前帧的pose,地图点固定. 
     * @param pFrame : 输入优化的关键帧
     */
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    /**
     * @brief 优化EssentialGraph图,主要用在 LoopClosing::CorrectLoop()这个函数
     * @param pMap: 地图
     * @param pLoopKF: 回环帧
     * @param pCurKF: 当前帧
     * @param NonCorrectedSim3: 没有纠正的关键帧
     * @param CorrectedSim3: 纠正后的关键帧
     * @param LoopConnections: 回环的关系
     * @param bFixScale: 是否固定缩放比例,if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
     */
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    /**
     * @brief 两针之间进行Sim3优化
     * @param pKF1:pKF1
     * @param pKF2:pKF2
     * @param vpMatches1:pKF2中匹配的特征点
     * @param g2oS12:两帧之间的Sim3位姿关系
     * @param th2:阈值
     * @param bFixScale: 是否固定缩放比例,if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
     */
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
