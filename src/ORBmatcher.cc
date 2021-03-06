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

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

    /// the threshold of descriptor, Haming dis less than threshold
    /// will be conside as correpondence keypoints
    const int ORBmatcher::TH_HIGH = 100;
    const int ORBmatcher::TH_LOW = 50;
    /// The histogram lenght, divide 0-360 degree to HISTO_LENGTH kinds uniform
    /// for example, if HISTO_LENGTH=3, then the angle 0-360 is divide to 0-120, 120-240, and 240-360.
    const int ORBmatcher::HISTO_LENGTH = 30;

    ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri)
    {
    }

    int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint *> &vpMapPoints, const float th)
    {
        int nmatches = 0;

        const bool bFactor = th != 1.0;

        ///loop all local MapPoint
        for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
        {
            MapPoint *pMP = vpMapPoints[iMP];
            if (!pMP->mbTrackInView)
                continue;

            if (pMP->isBad())
                continue;

            const int &nPredictedLevel = pMP->mnTrackScaleLevel;

            // The size of the window will depend on the viewing direction
            /// comfirm search radius according to view angle cos
            //搜索特征点的窗口大小取决于视场角度
            float r = RadiusByViewingCos(pMP->mTrackViewCos);

            if (bFactor)
                r *= th;

            /// get ketpoints in search radius and scale level range
            const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel], nPredictedLevel - 1, nPredictedLevel);

            if (vIndices.empty())
                continue;

            const cv::Mat MPdescriptor = pMP->GetDescriptor();

            int bestDist = 256;
            int bestLevel = -1;
            int bestDist2 = 256;
            int bestLevel2 = -1;
            int bestIdx = -1;

            // Get best and second matches with near keypoints
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;

                if (F.mvpMapPoints[idx])
                    if (F.mvpMapPoints[idx]->Observations() > 0)
                        continue;

                if (F.mvuRight[idx] > 0)
                {
                    const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                    if (er > r * F.mvScaleFactors[nPredictedLevel])
                        continue;
                }

                const cv::Mat &d = F.mDescriptors.row(idx);

                const int dist = DescriptorDistance(MPdescriptor, d);

                if (dist < bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestLevel2 = bestLevel;
                    bestLevel = F.mvKeysUn[idx].octave;
                    bestIdx = idx;
                }
                else if (dist < bestDist2)
                {
                    bestLevel2 = F.mvKeysUn[idx].octave;
                    bestDist2 = dist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if (bestDist <= TH_HIGH)
            {
                if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                    continue;
                //给特征点找到对应的MapPoint
                F.mvpMapPoints[bestIdx] = pMP;
                nmatches++;
            }
        }

        return nmatches;
    }

    float ORBmatcher::RadiusByViewingCos(const float &viewCos)
    {
        if (viewCos > 0.998)
            return 2.5;
        else
            return 4.0;
    }

    /// check kp1 and kp2 meet the F12 epipololar line constrant
    bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF2)
    {
        // Epipolar line in second image l = x1'F12 = [a b c]
        /// x1' * F12 * x2 = 0,  map kp1 to epipolor line in key frame 2
        const float a = kp1.pt.x * F12.at<float>(0, 0) + kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
        const float b = kp1.pt.x * F12.at<float>(0, 1) + kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
        const float c = kp1.pt.x * F12.at<float>(0, 2) + kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

        /// compute distance from keypoint to epipolar line
        // 计算kp2特征点到极线的距离：
        // 极线l：ax + by + c = 0
        // (u,v)到l的距离为： |au+bv+c| / sqrt(a^2+b^2)
        const float num = a * kp2.pt.x + b * kp2.pt.y + c;

        const float den = a * a + b * b;

        if (den == 0)
            return false;

        const float dsqr = num * num / den;

        return dsqr < 3.84 * pKF2->mvLevelSigma2[kp2.octave];
    }

    /**find the correspondence between pK's MapPoint and F' KeyPoint use DBOW */
    int ORBmatcher::SearchByBoW(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMapPointMatches)
    {
        /// get KeyFrame's MapPoint
        const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();

        vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

        int nmatches = 0;

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        /// DBoW2::FeatureVector == std::map<NodeId, std::vector<unsigned int>
        /// loop vFeatVecKF and  F.mFeatVec all node
        while (KFit != KFend && Fit != Fend)
        {
            /// (only the belong to the same DBOW node id, keypoint can be matched)
            if (KFit->first == Fit->first)
            {
                /// feature index
                const vector<unsigned int> vIndicesKF = KFit->second;
                const vector<unsigned int> vIndicesF = Fit->second;

                /// loop all keyframe features
                for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
                {
                    ///1.get keypoint's index
                    const unsigned int realIdxKF = vIndicesKF[iKF];

                    ///2.get feature's MapPoint
                    MapPoint *pMP = vpMapPointsKF[realIdxKF];

                    if (!pMP)
                        continue;

                    if (pMP->isBad())
                        continue;

                    ///3.get keypoints' descriptor
                    const cv::Mat &dKF = pKF->mDescriptors.row(realIdxKF);

                    int bestDist1 = 256;
                    int bestIdxF = -1;
                    int bestDist2 = 256;

                    /// loop all frame's keypoints
                    for (size_t iF = 0; iF < vIndicesF.size(); iF++)
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        /// not null, means this keypoints has been matched before,ignore
                        if (vpMapPointMatches[realIdxF])
                            continue;

                        const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                        /// compute the Haming distance between keyframe's descriptor and frame's descriptor
                        const int dist = DescriptorDistance(dKF, dF);

                        /// find the best two matched keypoint
                        if (dist < bestDist1)
                        {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF = realIdxF;
                        }
                        else if (dist < bestDist2)
                        {
                            bestDist2 = dist;
                        }
                    }

                    /// if best matche keypoint meet TH_LOW
                    if (bestDist1 <= TH_LOW)
                    {
                        /// if the best matched and second matched differ a lot, then match success
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            vpMapPointMatches[bestIdxF] = pMP;

                            const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];

                            /// check orientation
                            if (mbCheckOrientation)
                            {
                                float rot = kp.angle - F.mvKeys[bestIdxF].angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxF);
                            }
                            nmatches++;
                        }
                    }
                }

                KFit++;
                Fit++;
            }
            else if (KFit->first < Fit->first)
            {
                KFit = vFeatVecKF.lower_bound(Fit->first);
            }
            else
            {
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    int ORBmatcher::SearchByProjection(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, vector<MapPoint *> &vpMatched, int th)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw / scw;
        cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;
        cv::Mat Ow = -Rcw.t() * tcw;

        // Set of MapPoints already found in the KeyFrame
        set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
        spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

        int nmatches = 0;

        // For each Candidate MapPoint Project and Match
        for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++)
        {
            MapPoint *pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            if (pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            cv::Mat p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0)
                continue;

            // Project into Image
            const float invz = 1 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            // Depth must be inside the scale invariance region of the point
            //深度必须在点的比例不变区域内
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist = cv::norm(PO);

            if (dist < minDistance || dist > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;
                if (vpMatched[idx])
                    continue;

                const int &kpLevel = pKF->mvKeysUn[idx].octave;

                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_LOW)
            {
                vpMatched[bestIdx] = pMP;
                nmatches++;
            }
        }

        return nmatches;
    }

    int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
    {
        int nmatches = 0;
        vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

        //宽度为HISTO_LENGTH的直方图
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        //匹配的描述子之间的距离
        vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
        //储存的是F2和F1的匹配的特征点之间对应的索引ID,通过F2的特征点ID可以得到F1的特征点ID
        vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

        //遍历F1的特征点,为F1的每一个特征点在F2中找到一个最相近的特征点
        for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++)
        {
            cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            int level1 = kp1.octave;
            if (level1 > 0)
                continue;

            ///将F1的特征点的位置作为在F2中搜索的初始位置,并搜索在windowSize范围内的特征点
            vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

            if (vIndices2.empty())
                continue;

            //F1特征点对应的描述子
            cv::Mat d1 = F1.mDescriptors.row(i1);

            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

             //为F1的特征点在F2中找到一个最相近的特征点
            for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
            {
                size_t i2 = *vit;

                //F2特征点对应的描述子
                cv::Mat d2 = F2.mDescriptors.row(i2);

                //计算两个描述子之间的距离
                int dist = DescriptorDistance(d1, d2);

                //如果计算得到的距离小于已经储存的距离,就排除F2中的这个特征点
                //说明F2中这个特征点已经在F1中找到更合适的匹配点了
                if (vMatchedDistance[i2] <= dist)
                    continue;

                //找到最小的两个匹配距离,和F2中最佳的匹配特征点
                if (dist < bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestIdx2 = i2;
                }
                else if (dist < bestDist2)
                {
                    bestDist2 = dist;
                }
            }//for

            //最佳匹配点的匹配距离满足阈值
            if (bestDist <= TH_LOW)
            {
                //最小距离比次小的距离小的比较明显
                if (bestDist < (float)bestDist2 * mfNNratio)
                {
                    //如果F2:bestIdx2已经有对应的F1匹配特征点,首先删除这个对应连接
                    if (vnMatches21[bestIdx2] >= 0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]] = -1;
                        nmatches--;
                    }
                    //储存匹配ID的在F1和F2之间的对应关系
                    vnMatches12[i1] = bestIdx2;
                    vnMatches21[bestIdx2] = i1;
                    vMatchedDistance[bestIdx2] = bestDist;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        //特征点的方向差
                        float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        //将F1的特征点i1加入对应的直方图格子
                        rotHist[bin].push_back(i1);
                    }
                }
            }
        }

        //如果需要匹配旋转角度
        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ///得到三个最大的值
            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if (vnMatches12[idx1] >= 0)
                    {
                        vnMatches12[idx1] = -1;
                        nmatches--;
                    }
                }
            }
        }

        //Update prev matched
        for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
            if (vnMatches12[i1] >= 0)
                vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

        return nmatches;
    }

    int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
    {
        const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
        /**
         * DBoW2::FeatureVector的数据类型 std::map<NodeId, std::vector<unsigned int> >
         * 储存着NodeId和特征点在Frame的索引值
         */
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        const cv::Mat &Descriptors1 = pKF1->mDescriptors;

        const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
        const cv::Mat &Descriptors2 = pKF2->mDescriptors;

        vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
        vector<bool> vbMatched2(vpMapPoints2.size(), false);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / HISTO_LENGTH;

        int nmatches = 0;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        //遍历vFeatVec1的所有节点
        while (f1it != f1end && f2it != f2end)
        {
            //首先判断NodeId是否一样,即在同一个Node中,只有同一个Node特征点才有可能匹配
            if (f1it->first == f2it->first)
            {
                //遍历pKF1这个Node中的所有的特征点
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
                {
                    //取得特征点在pKF1关键帧中的索引
                    const size_t idx1 = f1it->second[i1];
                    //得到对应的地图点
                    MapPoint *pMP1 = vpMapPoints1[idx1];
                    if (!pMP1)
                        continue;
                    if (pMP1->isBad())
                        continue;

                    //Descriptors1中一行代表一个特征点的描述子,获取描述子
                    const cv::Mat &d1 = Descriptors1.row(idx1);

                    int bestDist1 = 256;
                    int bestIdx2 = -1;
                    int bestDist2 = 256;

                    //遍历pKF2这个Node中的所有的特征点
                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)
                    {
                        const size_t idx2 = f2it->second[i2];

                        MapPoint *pMP2 = vpMapPoints2[idx2];

                        if (vbMatched2[idx2] || !pMP2)
                            continue;

                        if (pMP2->isBad())
                            continue;

                        const cv::Mat &d2 = Descriptors2.row(idx2);
                        //计算描述子距离
                        int dist = DescriptorDistance(d1, d2);

                        if (dist < bestDist1)
                        {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdx2 = idx2;
                        }
                        else if (dist < bestDist2)
                        {
                            bestDist2 = dist;
                        }
                    }

                    //如果小于阈值
                    if (bestDist1 < TH_LOW)
                    {
                        //bestDist1比bestDist2小的明显
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2))
                        {
                            //匹配成功,pKF1中的第idx1个特征点对应的MapPoint为pKF1中的第bestIdx2的MapPoint
                            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2] = true;

                            if (mbCheckOrientation)
                            {
                                float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            //如果f1it->first小于f2it->first
            else if (f1it->first < f2it->first)
            {   //在vFeatVec1中找到第一个不小于f2it->first的NodeId
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }//while

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }

        return nmatches;
    }

    /// Find matches between not tracked keypoints,Matching speed-up by ORB Vocabulary,
    /// Compare only ORB that share the same node
    int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                           vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo)
    {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        //Compute epipole in second image
        /// camera center in world frame
        cv::Mat Cw = pKF1->GetCameraCenter();
        /// pKF2 rotation and translation
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();
        /// pKF1's camera center pose in pKF2 frame
        cv::Mat C2 = R2w * Cw + t2w;
        /// 将pKF1的光心投影到pKF2的像素平面,得到极点(ex,ey)
        const float invz = 1.0f / C2.at<float>(2);
        const float ex = pKF2->fx * C2.at<float>(0) * invz + pKF2->cx;
        const float ey = pKF2->fy * C2.at<float>(1) * invz + pKF2->cy;

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches = 0;
        vector<bool> vbMatched2(pKF2->N, false);
        vector<int> vMatches12(pKF1->N, -1);

        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = 1.0f / HISTO_LENGTH;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while (f1it != f1end && f2it != f2end)
        {
            if (f1it->first == f2it->first)
            {
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint *pMP1 = pKF1->GetMapPoint(idx1);

                    // If there is already a MapPoint skip
                    if (pMP1)
                        continue;

                    const bool bStereo1 = pKF1->mvuRight[idx1] >= 0;

                    if (bOnlyStereo)
                        if (!bStereo1)
                            continue;

                    const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];

                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)
                    {
                        size_t idx2 = f2it->second[i2];

                        MapPoint *pMP2 = pKF2->GetMapPoint(idx2);

                        // If we have already matched or there is a MapPoint skip
                        if (vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = pKF2->mvuRight[idx2] >= 0;

                        if (bOnlyStereo)
                            if (!bStereo2)
                                continue;

                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        const int dist = DescriptorDistance(d1, d2);

                        if (dist > TH_LOW || dist > bestDist)
                            continue;

                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                        /// not stereo frame
                        if (!bStereo1 && !bStereo2)
                        {
                            //计算特征点到极点的距离的平方,该特征点距离极点太近，表明kp2对应的MapPoint距离pKF1相机太近，所以也要剔除
                            const float distex = ex - kp2.pt.x;
                            const float distey = ey - kp2.pt.y;
                            if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave])
                                continue;
                        }

                        if (CheckDistEpipolarLine(kp1, kp2, F12, pKF2))
                        {
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }
                    
                    if (bestIdx2 >= 0)
                    {
                        //匹配成功
                        const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                        vMatches12[idx1] = bestIdx2;
                        nmatches++;

                        if (mbCheckOrientation)
                        {
                            float rot = kp1.angle - kp2.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if (f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    vMatches12[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }
        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
        {
            if (vMatches12[i] < 0)
                continue;
            vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
        }

        return nmatches;
    }

    int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
    {
        cv::Mat Rcw = pKF->GetRotation();
        cv::Mat tcw = pKF->GetTranslation();

        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;
        const float &bf = pKF->mbf;

        cv::Mat Ow = pKF->GetCameraCenter();

        int nFused = 0;

        const int nMPs = vpMapPoints.size();

        /// 1.project the map point to pKF image plane, we get pixel (u, v);
        /// 2.get all center(u,v),radius range feature in pKF image plane;
        /// 3.find the best feature match the map point
        for (int i = 0; i < nMPs; i++)
        {
            MapPoint *pMP = vpMapPoints[i];

            /// invalid pointer
            if (!pMP)
                continue;

            /// is MapPoint bad and MapPoint can be seen by KeyFrame
            if (pMP->isBad() || pMP->IsInKeyFrame(pKF))
                continue;

            /// MapPoint 3D Pose in world coordinate
            cv::Mat p3Dw = pMP->GetWorldPos();
            /// transform MapPoint to camera coordinate
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0f)
                continue;

            const float invz = 1 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            const float ur = u - bf * invz;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist3D = cv::norm(PO);

            // Depth must be inside the scale pyramid of the image
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist3D)
                continue;

            /// according dis3d and pKf to predict pyramid scale level
            int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

                const int &kpLevel = kp.octave;

                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                /// stereo camera
                if (pKF->mvuRight[idx] >= 0)
                {
                    // Check reprojection error in stereo
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float &kpr = pKF->mvuRight[idx];
                    const float ex = u - kpx;
                    const float ey = v - kpy;
                    const float er = ur - kpr;
                    const float e2 = ex * ex + ey * ey + er * er;

                    if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8)
                        continue;
                }
                /// monocular
                else
                {
                    const float &kpx = kp.pt.x;
                    const float &kpy = kp.pt.y;
                    const float ex = u - kpx;
                    const float ey = v - kpy;
                    const float e2 = ex * ex + ey * ey;

                    if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99)
                        continue;
                }

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            /// if there is already a MapPoint correspondence to the feature, then
            if (bestDist <= TH_LOW)
            {
                MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
                if (pMPinKF)
                {
                    if (!pMPinKF->isBad())
                    {
                        /// pMPinKF have more key frame observation than pMp
                        if (pMPinKF->Observations() > pMP->Observations())
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                }
                else
                {
                    /// map point add pKF as observation
                    pMP->AddObservation(pKF, bestIdx);
                    /// key frame add map point
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

    int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
    {
        // Get Calibration Parameters for later projection
        const float &fx = pKF->fx;
        const float &fy = pKF->fy;
        const float &cx = pKF->cx;
        const float &cy = pKF->cy;

        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw / scw;
        cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;
        cv::Mat Ow = -Rcw.t() * tcw;

        // Set of MapPoints already found in the KeyFrame
        const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();

        int nFused = 0;

        const int nPoints = vpPoints.size();

        // For each candidate MapPoint project and match
        for (int iMP = 0; iMP < nPoints; iMP++)
        {
            MapPoint *pMP = vpPoints[iMP];

            // Discard Bad MapPoints and already found
            if (pMP->isBad() || spAlreadyFound.count(pMP))
                continue;

            // Get 3D Coords.
            cv::Mat p3Dw = pMP->GetWorldPos();

            // Transform into Camera Coords.
            cv::Mat p3Dc = Rcw * p3Dw + tcw;

            // Depth must be positive
            if (p3Dc.at<float>(2) < 0.0f)
                continue;

            // Project into Image
            const float invz = 1.0 / p3Dc.at<float>(2);
            const float x = p3Dc.at<float>(0) * invz;
            const float y = p3Dc.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF->IsInImage(u, v))
                continue;

            // Depth must be inside the scale pyramid of the image
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - Ow;
            const float dist3D = cv::norm(PO);

            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if (PO.dot(Pn) < 0.5 * dist3D)
                continue;

            // Compute predicted scale level
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

            // Search in a radius
            const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++)
            {
                const size_t idx = *vit;
                const int &kpLevel = pKF->mvKeysUn[idx].octave;

                if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mDescriptors.row(idx);

                int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            if (bestDist <= TH_LOW)
            {
                MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
                if (pMPinKF)
                {
                    if (!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                }
                else
                {
                    pMP->AddObservation(pKF, bestIdx);
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }

        return nFused;
    }

    int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12,
                                 const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
    {
        const float &fx = pKF1->fx;
        const float &fy = pKF1->fy;
        const float &cx = pKF1->cx;
        const float &cy = pKF1->cy;

        // Camera 1 from world
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();

        //Camera 2 from world
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        //Transformation between cameras
        cv::Mat sR12 = s12 * R12;
        cv::Mat sR21 = (1.0 / s12) * R12.t();
        cv::Mat t21 = -sR21 * t12;

        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        const int N1 = vpMapPoints1.size();

        const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
        const int N2 = vpMapPoints2.size();

        vector<bool> vbAlreadyMatched1(N1, false);
        vector<bool> vbAlreadyMatched2(N2, false);

        for (int i = 0; i < N1; i++)
        {
            MapPoint *pMP = vpMatches12[i];
            if (pMP)
            {
                vbAlreadyMatched1[i] = true;
                int idx2 = pMP->GetIndexInKeyFrame(pKF2);
                if (idx2 >= 0 && idx2 < N2)
                    vbAlreadyMatched2[idx2] = true;
            }
        }

        vector<int> vnMatch1(N1, -1);
        vector<int> vnMatch2(N2, -1);

        // Transform from KF1 to KF2 and search
        for (int i1 = 0; i1 < N1; i1++)
        {
            MapPoint *pMP = vpMapPoints1[i1];

            if (!pMP || vbAlreadyMatched1[i1])
                continue;

            if (pMP->isBad())
                continue;

            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Dc1 = R1w * p3Dw + t1w;
            cv::Mat p3Dc2 = sR21 * p3Dc1 + t21;

            // Depth must be positive
            if (p3Dc2.at<float>(2) < 0.0)
                continue;

            const float invz = 1.0 / p3Dc2.at<float>(2);
            const float x = p3Dc2.at<float>(0) * invz;
            const float y = p3Dc2.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF2->IsInImage(u, v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = cv::norm(p3Dc2);

            // Depth must be inside the scale invariance region
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

            // Search in a radius
            const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

                if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_HIGH)
            {
                vnMatch1[i1] = bestIdx;
            }
        }

        // Transform from KF2 to KF1 and search
        for (int i2 = 0; i2 < N2; i2++)
        {
            MapPoint *pMP = vpMapPoints2[i2];

            if (!pMP || vbAlreadyMatched2[i2])
                continue;

            if (pMP->isBad())
                continue;

            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Dc2 = R2w * p3Dw + t2w;
            cv::Mat p3Dc1 = sR12 * p3Dc2 + t12;

            // Depth must be positive
            if (p3Dc1.at<float>(2) < 0.0)
                continue;

            const float invz = 1.0 / p3Dc1.at<float>(2);
            const float x = p3Dc1.at<float>(0) * invz;
            const float y = p3Dc1.at<float>(1) * invz;

            const float u = fx * x + cx;
            const float v = fy * y + cy;

            // Point must be inside the image
            if (!pKF1->IsInImage(u, v))
                continue;

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const float dist3D = cv::norm(p3Dc1);

            // Depth must be inside the scale pyramid of the image
            if (dist3D < minDistance || dist3D > maxDistance)
                continue;

            // Compute predicted octave
            const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

            // Search in a radius of 2.5*sigma(ScaleLevel)
            const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

            if (vIndices.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = INT_MAX;
            int bestIdx = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
            {
                const size_t idx = *vit;

                const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

                if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

                const int dist = DescriptorDistance(dMP, dKF);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = idx;
                }
            }

            if (bestDist <= TH_HIGH)
            {
                vnMatch2[i2] = bestIdx;
            }
        }

        // Check agreement
        int nFound = 0;

        for (int i1 = 0; i1 < N1; i1++)
        {
            int idx2 = vnMatch1[i1];

            if (idx2 >= 0)
            {
                int idx1 = vnMatch2[idx2];
                if (idx1 == i1)
                {
                    vpMatches12[i1] = vpMapPoints2[idx2];
                    nFound++;
                }
            }
        }

        return nFound;
    }

    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    {
        int nmatches = 0;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);

        /// tcw --> twc
        /// twc + Rwc*tcw = 0
        /// ==> twc + Rcw.t()*tcw = 0
        /// ==> twc = -Rcw.t()*tcw
        const cv::Mat twc = -Rcw.t() * tcw;

        const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);

        /// translate last frame to current frame
        const cv::Mat tlc = Rlw * twc + tlw;

        const bool bForward = tlc.at<float>(2) > CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc.at<float>(2) > CurrentFrame.mb && !bMono;

        /// loop lastFrame's all keypoints
        for (int i = 0; i < LastFrame.N; i++)
        {
            MapPoint *pMP = LastFrame.mvpMapPoints[i];

            /// check map pointer valid
            if (pMP)
            {
                /// check map point is inlier
                if (!LastFrame.mvbOutlier[i])
                {
                    // Project
                    /// get map point's pose ref world frame
                    cv::Mat x3Dw = pMP->GetWorldPos();
                    /// transform the map point to current frame
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;

                    const float xc = x3Dc.at<float>(0);
                    const float yc = x3Dc.at<float>(1);
                    const float invzc = 1.0 / x3Dc.at<float>(2);

                    if (invzc < 0)
                        continue;

                    float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                    float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                    if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
                        continue;
                    if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
                        continue;

                    int nLastOctave = LastFrame.mvKeys[i].octave;

                    // Search in a window. Size depends on scale
                    float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    /// get keypoints index from scale nLastOctave to max level in radius size windows
                    if (bForward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave);
                    /// get keypoints index from scale 0 to nLastOctave scale in  radius size windows
                    else if (bBackward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, 0, nLastOctave);
                    else
                        vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave - 1, nLastOctave + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    /// find keypoint in current frame closest to this map point through compare Haming distance
                    for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend; vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMapPoints[i2])
                            if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0)
                                continue;

                        if (CurrentFrame.mvuRight[i2] > 0)
                        {
                            const float ur = u - CurrentFrame.mbf * invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if (er > radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    /// if Haming dis less than TH_HIGH, then the map point find the corresponce keypoints in current frame
                    if (bestDist <= TH_HIGH)
                    {
                        /// construct correspondence between the map point and the current frame's keypoint
                        CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                        /// match number plus 1
                        nmatches++;

                        /// check orientation
                        if (mbCheckOrientation)
                        {
                            /// diff angle between current keypoint's angle and correspondence's keypoint's angle
                            /// in last frame
                            float rot = LastFrame.mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                            /// converte to 0-360
                            if (rot < 0.0)
                                rot += 360.0f;
                            /// compute the diff angle belong to which bin in the histogram
                            /// i think there is a bug, if HISTO_LENGTH=3, then factor=1.0/3
                            /// if rot = 100(deg), then bin = 33, at this time bin is large
                            /// than HISTO_LENGTH. or HISTO_LENGTH must meet HISTO_LENGTH^2
                            /// >= 360, ==> HISTO_LENGTH >= 19
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            /// bin range 0 - HISTO_LENGTH
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                }
            }
        }

        //Apply rotation consistency
        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            /// most keypoints's diff angle belong to this three bin index
            /// if not, the keypoints corresondence may be incorrect
            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                //特征点最多的三个bin都不是rotHist[i]这个bin
                if (i != ind1 && i != ind2 && i != ind3)
                {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint *> &sAlreadyFound, const float th, const int ORBdist)
    {
        int nmatches = 0;

        //得到在相机坐标系下,当前帧的光心在世界坐标系原点到的坐标Ow
        /// tcw --> twc
        /// twc + Rwc*tcw = 0
        /// ==> twc + Rcw.t()*tcw = 0
        /// ==> twc = -Rcw.t()*tcw
        /// Ow == twc
        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);//在CurrentFrame坐标系下,光心相对于世界坐标系原点的坐标
        const cv::Mat Ow = -Rcw.t() * tcw;//在世界坐标系下,光心相对于世界坐标系的坐标

        // Rotation Histogram (to check rotation consistency)  
        //旋转直方图,用来检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if (pMP)
            {
                if (!pMP->isBad() && !sAlreadyFound.count(pMP))
                {
                    //Project
                    //将3D MapPoint投影到camera坐标系
                    cv::Mat x3Dw = pMP->GetWorldPos();
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;

                    const float xc = x3Dc.at<float>(0);
                    const float yc = x3Dc.at<float>(1);
                    const float invzc = 1.0 / x3Dc.at<float>(2);

                    const float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                    const float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                    if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
                        continue;
                    if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
                        continue;

                    // Compute predicted scale level
                    //在世界坐标系下,计算光心到3D MapPoint的向量
                    cv::Mat PO = x3Dw - Ow;
                    float dist3D = cv::norm(PO);

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // Depth must be inside the scale pyramid of the image
                    //Depth必须在图像金字塔的缩放范围内
                    if (dist3D < minDistance || dist3D > maxDistance)
                        continue;

                    int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

                    // Search in a window
                    const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel - 1, nPredictedLevel + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMapPoints[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= ORBdist)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                        nmatches++;

                        if (mbCheckOrientation)
                        {
                            float rot = pKF->mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                }
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i != ind1 && i != ind2 && i != ind3)
                {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]] = NULL;
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++)
        {
            const int s = histo[i].size();
            if (s > max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            }
            else if (s > max2)
            {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            }
            else if (s > max3)
            {
                max3 = s;
                ind3 = i;
            }
        }

        //如果第二大的值远远小于第一大的值,则将索引ind2和ind3设置为-1
        if (max2 < 0.1f * (float)max1)
        {
            ind2 = -1;
            ind3 = -1;
        }
        //如果第三大的值远远小于第一大的值,则将索引ind3设置为-1
        else if (max3 < 0.1f * (float)max1)
        {
            ind3 = -1;
        }
    }

    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    /**
     * 计算两个描述子之间的汉明距离,即不同位的个数
     * 描述子为256位,这里用了位运算的一些技巧
     * 0x55555555->0101 0101 0101 0101
     * 0x33333333->0011 0011 0011 0011
     */
    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

} // namespace ORB_SLAM2
