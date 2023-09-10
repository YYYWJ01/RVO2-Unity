/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;

namespace RVO
{
    /**
     * <summary>在模拟中定义代理。</summary>
     */
    internal class Agent
    {
        /// <summary>
        /// 代理邻居列表<距离，代理对象>
        /// PS：按照距离由近到远依次排列
        /// </summary>
        /// <returns></returns>
        internal IList<KeyValuePair<float, Agent>> agentNeighbors_ = new List<KeyValuePair<float, Agent>>();
        /// <summary>
        /// 障碍邻居列表<距离，障碍物对象>
        /// PS：按照距离由近到远依次排列
        /// </summary>
        /// <returns></returns>
        internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<float, Obstacle>>();
        /// <summary>
        /// OCRA线列表
        /// </summary>
        /// <typeparam name="Line"></typeparam>
        /// <returns></returns>
        internal IList<Line> orcaLines_ = new List<Line>();
        /// <summary>
        /// 代理位置
        /// </summary>
        internal Vector2 position_;
        /// <summary>
        /// 首选速度
        /// </summary>
        internal Vector2 prefVelocity_;
        /// <summary>
        /// 新代理的默认初始二维线速度。
        /// </summary>
        internal Vector2 velocity_;
        /// <summary>
        /// 代理Id
        /// </summary>
        internal int id_ = 0;
        /// <summary>
        /// 新代理在导航中考虑的其他代理的默认最大数量。该数字越大，模拟的运行时间越长。 如果数字太低，模拟将不安全。
        /// </summary>
        internal int maxNeighbors_ = 0;
        /// <summary>
        /// 新代理的默认最大速度。 必须是非负数。
        /// </summary>
        internal float maxSpeed_ = 0.0f;
        /// <summary>
        /// 新代理在导航中考虑到其他代理的默认最大距离（中心点到中心点）。这个数字越大，模拟的运行时间就越长。 如果数字太低，模拟将不安全。 必须是非负数。
        /// </summary>
        internal float neighborDist_ = 0.0f;
        /// <summary>
        /// 新代理的默认半径。 必须是非负数。
        /// </summary>
        internal float radius_ = 0.0f;
        /// <summary>
        /// 通过模拟计算的新代理速度相对于其他代理来说是安全的默认最短时间。这个数字越大，代理对其他代理的存在做出响应的速度就越快，但代理选择其速度的自由度就越小。必须是正向的。
        /// </summary>
        internal float timeHorizon_ = 0.0f;
        /// <summary>
        /// 通过模拟计算的新代理速度相对于障碍物是安全的默认最短时间。这个数字越大，智能体对障碍物的存在响应越快，但智能体选择速度的自由度就越小。必须是正向的。
        /// </summary>
        internal float timeHorizonObst_ = 0.0f;
        /// <summary>
        /// 需要删除状态
        /// </summary>
        internal bool needDelete_ = false;
        /// <summary>
        /// 新速度
        /// </summary>
        private Vector2 newVelocity_;

        /**
         * <summary>计算该代理的邻居。</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors_.Clear();
            float rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
            Simulator.Instance.kdTree_.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors_.Clear();

            if (maxNeighbors_ > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist_);
                Simulator.Instance.kdTree_.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>计算这个代理的新速度。</summary>
         */
        internal void computeNewVelocity()
        {
            // 清空存储ORCA（Optimal Reciprocal Collision Avoidance）线的列表，以准备计算新的ORCA线。
            orcaLines_.Clear();

            // 计算与障碍物互动的时间视界的倒数。timeHorizonObst_ 通常表示为代理与障碍物之间的最大交互时间。
            float invTimeHorizonObst = 1.0f / timeHorizonObst_;

            /* 制造障碍ORCA线。 */
            // 处理与代理相邻的障碍物。
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {
                // 获取与代理相邻的障碍物中的当前障碍物。
                Obstacle obstacle1 = obstacleNeighbors_[i].Value;
                // 获取与代理相邻的障碍物中的当前障碍物的下一个障碍物，通常障碍物是按顺序连接的。
                Obstacle obstacle2 = obstacle1.next_;

                // 计算代理与障碍物1之间的相对位置矢量。
                Vector2 relativePosition1 = obstacle1.point_ - position_;
                // 计算代理与障碍物2之间的相对位置矢量。
                Vector2 relativePosition2 = obstacle2.point_ - position_;

                /*
                 * 检查障碍物的速度障碍是否已经被先前构建的障碍物ORCA线处理过。
                 */
                // 用于标记障碍物是否已经被先前构建的ORCA线处理过。
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines_.Count; ++j)
                {
                    // 通过对已存在的ORCA线进行检查，以确定是否已经处理了障碍物。
                    if (RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON && RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                // 用于标记障碍物是否已经被先前构建的ORCA线处理过。
                if (alreadyCovered)
                {
                    continue;
                }

                /* 尚未覆盖。检查碰撞。 */
                // 计算代理与障碍物1之间的距离的平方。
                float distSq1 = RVOMath.absSq(relativePosition1);
                // // 计算代理与障碍物1之间的距离的平方。
                float distSq2 = RVOMath.absSq(relativePosition2);

                // 计算代理的半径 radius_ 的平方。
                float radiusSq = RVOMath.sqr(radius_);

                // 计算障碍物1到障碍物2的矢量 obstacleVector，这是用来描述障碍物边的方向的矢量。
                Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
                // 计算代理相对于障碍物边的投影参数 s。这个参数用于表示代理在障碍物上的投影点的位置。
                // 如果 s 的值小于0，代表代理在障碍物1的左侧，如果 s 的值大于1，代表代理在障碍物2的左侧，如果 s 的值在0到1之间，代表代理在障碍物边之间。
                float s = (-relativePosition1 * obstacleVector) / RVOMath.absSq(obstacleVector);
                // 计算代理与障碍物边之间的距离的平方，即代理与障碍物边之间的最短距离的平方。这个值用于判断代理是否与障碍物边相交或接近。
                float distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

                // 声明一个 Line 结构，用于存储ORCA线的信息。
                Line line;

                if (s < 0.0f && distSq1 <= radiusSq)
                {
                    /* 与左顶点的碰撞。忽略非凸。 */
                    if (obstacle1.convex_)
                    {
                        line.point = new Vector2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y(), relativePosition1.x()));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0f && distSq2 <= radiusSq)
                {
                    /*
                     * 与右顶点的碰撞。如果它是非凸的，或者如果它会被邻近的障碍物处理，请忽略。
                     */
                    if (obstacle2.convex_ && RVOMath.det(relativePosition2, obstacle2.direction_) >= 0.0f)
                    {
                        line.point = new Vector2(0.0f, 0.0f);
                        line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y(), relativePosition2.x()));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq)
                {
                    /* 与障碍物段的碰撞。 */
                    line.point = new Vector2(0.0f, 0.0f);
                    line.direction = -obstacle1.direction_;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * 没有碰撞。 计算腿。 当倾斜观察时，两条腿可以来自同一个顶点。 当非凸顶点时，腿延伸截止线。
                 */

                // 用于表示代理的左腿和右腿的方向。左腿是指代理相对于障碍物1的腿，右腿是指代理相对于障碍物2的腿。
                Vector2 leftLegDirection, rightLegDirection;

                if (s < 0.0f && distSqLine <= radiusSq)
                {
                    /*
                     * 倾斜观察障碍物，以便左顶点定义速度障碍物。
                     */
                    if (!obstacle1.convex_)
                    {
                        /* 忽略障碍。 */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
                    rightLegDirection = new Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
                }
                else if (s > 1.0f && distSqLine <= radiusSq)
                {
                    /*
                     * 倾斜地观察障碍物，以便右顶点定义速度障碍物。
                     */
                    if (!obstacle2.convex_)
                    {
                        /* 忽略障碍。 */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
                    rightLegDirection = new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
                }
                else
                {
                    /* 通常情况。 */
                    if (obstacle1.convex_)
                    {
                        float leg1 = RVOMath.sqrt(distSq1 - radiusSq);
                        leftLegDirection = new Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
                    }
                    else
                    {
                        /* 左顶点非凸； 左腿延伸截止线。 */
                        leftLegDirection = -obstacle1.direction_;
                    }

                    if (obstacle2.convex_)
                    {
                        float leg2 = RVOMath.sqrt(distSq2 - radiusSq);
                        rightLegDirection = new Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
                    }
                    else
                    {
                        /* 右顶点非凸； 右腿延伸截止线。 */
                        rightLegDirection = obstacle1.direction_;
                    }
                }

                /*
                 * 当凸顶点时，腿不能指向相邻边，而是取相邻边的截止线。 如果速度投射在“外”腿上，则不添加任何约束。
                 */

                Obstacle leftNeighbor = obstacle1.previous_;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex_ && RVOMath.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0f)
                {
                    /* 左腿指向障碍物。 */
                    leftLegDirection = -leftNeighbor.direction_;
                    isLeftLegForeign = true;
                }

                if (obstacle2.convex_ && RVOMath.det(rightLegDirection, obstacle2.direction_) <= 0.0f)
                {
                    /* 右腿指向障碍物。 */
                    rightLegDirection = obstacle2.direction_;
                    isRightLegForeign = true;
                }

                /* 计算截止中心。 */
                Vector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
                Vector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
                Vector2 cutOffVector = rightCutOff - leftCutOff;

                /* 将当前速度投影到速度障碍物上。 */

                /* 检查当前速度是否投影在截止圆上。 */
                float t = obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutOff) * cutOffVector) / RVOMath.absSq(cutOffVector);
                float tLeft = (velocity_ - leftCutOff) * leftLegDirection;
                float tRight = (velocity_ - rightCutOff) * rightLegDirection;

                if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f))
                {
                    /* 投影在左侧截止圆上。 */
                    Vector2 unitW = RVOMath.normalize(velocity_ - leftCutOff);

                    line.direction = new Vector2(unitW.y(), -unitW.x());
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }
                else if (t > 1.0f && tRight < 0.0f)
                {
                    /* 投影在右截止圆上。 */
                    Vector2 unitW = RVOMath.normalize(velocity_ - rightCutOff);

                    line.direction = new Vector2(unitW.y(), -unitW.x());
                    line.point = rightCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * 投射到左腿、右腿或明暗截止线上，以最接近速度的为准。
                 */
                float distSqCutoff = (t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? float.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + t * cutOffVector));
                float distSqLeft = tLeft < 0.0f ? float.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + tLeft * leftLegDirection));
                float distSqRight = tRight < 0.0f ? float.PositiveInfinity : RVOMath.absSq(velocity_ - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* 计划在截止线上。 */
                    line.direction = -obstacle1.direction_;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
                    orcaLines_.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* 计划在左腿上。 */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
                    orcaLines_.Add(line);

                    continue;
                }

                /* 计划在右腿上。 */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y(), line.direction.x());
                orcaLines_.Add(line);
            }

            int numObstLines = orcaLines_.Count;

            float invTimeHorizon = 1.0f / timeHorizon_;

            /* 创建 ORCA 代理线。 */
            for (int i = 0; i < agentNeighbors_.Count; ++i)
            {
                Agent other = agentNeighbors_[i].Value;

                Vector2 relativePosition = other.position_ - position_;
                Vector2 relativeVelocity = velocity_ - other.velocity_;
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadius = radius_ + other.radius_;
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                Vector2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* 没有碰撞。 */
                    Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* 从截止中心到相对速度的矢量。 */
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = w * relativePosition;

                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* 投影在截止圆上。 */
                        float wLength = RVOMath.sqrt(wLengthSq);
                        Vector2 unitW = w / wLength;

                        line.direction = new Vector2(unitW.y(), -unitW.x());
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* 计划在腿上。 */
                        float leg = RVOMath.sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.det(relativePosition, w) > 0.0f)
                        {
                            /* 计划在左腿上。 */
                            line.direction = new Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                        }
                        else
                        {
                            /* 项目在右腿上。 */
                            line.direction = -new Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
                        }

                        float dotProduct2 = relativeVelocity * line.direction;
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* 碰撞。 投影时间 timeStep 的截止圈。 */
                    float invTimeStep = 1.0f / Simulator.Instance.timeStep_;

                    /* 从截止中心到相对速度的矢量。 */
                    Vector2 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = RVOMath.abs(w);
                    Vector2 unitW = w / wLength;

                    line.direction = new Vector2(unitW.y(), -unitW.x());
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = velocity_ + 0.5f * u;
                orcaLines_.Add(line);
            }

            int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
            }
        }

        /**
         * <summary>将代理邻居插入到该代理的邻居集中。</summary>
         *
         * <param name="agent">指向要插入的代理的指针。</param>
         * <param name="rangeSq">该主体周围的平方范围。</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.absSq(position_ - agent.position_);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors_.Count < maxNeighbors_)
                    {
                        agentNeighbors_.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors_.Count - 1;

                    while (i != 0 && distSq < agentNeighbors_[i - 1].Key)
                    {
                        agentNeighbors_[i] = agentNeighbors_[i - 1];
                        --i;
                    }

                    agentNeighbors_[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    // 如果当前代理邻居数据 等于 最大代理邻居数量，则将该代理周围的平方范围设置为 最远代理邻居的距离
                    if (agentNeighbors_.Count == maxNeighbors_)
                    {
                        rangeSq = agentNeighbors_[agentNeighbors_.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>将静态障碍物邻居插入到该代理的邻居集中。</summary>
         *
         * <param name="obstacle">需要插入的静态障碍物。</param>
         * <param name="rangeSq">代理与障碍物之间的最大距离的平方。</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle.next_;

            // 计算代理与给定障碍物 obstacle 所形成的线段的距离的平方。
            float distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            // 代理与该障碍物足够近，可以考虑将其添加到邻近障碍物列表中。
            if (distSq < rangeSq)
            {
                obstacleNeighbors_.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors_.Count - 1;

                // 将添加的障碍物按距离的平方从小到大排序，以便后续可以更轻松地找到最近的邻近障碍物。循环会持续直到找到合适的位置或遍历完整个列表。
                while (i != 0 && distSq < obstacleNeighbors_[i - 1].Key)
                {
                    // 如果当前障碍物的距离更小，就将前一个障碍物往后移动一位。
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                // 将当前障碍物插入到适当的位置，以确保列表中的障碍物按距离的平方从小到大排序。
                obstacleNeighbors_[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>更新该代理的二维位置和二维速度。</summary>
         */
        internal void update()
        {
            velocity_ = newVelocity_;
            position_ += velocity_ * Simulator.Instance.timeStep_;
        }

        /**
         * <summary>在受直线和圆形约束定义的线性约束约束下，求解指定直线上的一维线性规划。</summary>
         *
         * <returns>如果成功则为 true 。</returns>
         *
         * <param name="lines">定义线性约束的线。</param>
         * <param name="lineNo">指定的线约束。</param>
         * <param name="radius">圆形约束的半径。</param>
         * <param name="optVelocity">优化速度。</param>
         * <param name="directionOpt">如果应该优化方向则为 true。</param>
         * <param name="result">对线性程序结果的引用。</param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            float dotProduct = lines[lineNo].point * lines[lineNo].direction;
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0f)
            {
                /* 最大速度圈使线路 lineNo 完全失效。 */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                float denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                float numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (RVOMath.fabs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* 线 lineNo 和 i （几乎）平行。 */
                    if (numerator < 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* 第 i 行将右侧的第 lineNo 行作为边界。 */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* 第 i 行将左侧的第 lineNo 行作为边界。 */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* 优化方向。 */
                if (optVelocity * lines[lineNo].direction > 0.0f)
                {
                    /* 采取右极端。 */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* 采取左极端。 */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* 优化最近点。 */
                float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>求解受直线和圆形约束定义的线性约束影响的二维线性规划。</summary>
         *
         * <returns>失败的行数以及成功的行数。</returns>
         *
         * <param name="lines">定义线性约束的线。</param>
         * <param name="radius">圆形约束的半径。</param>
         * <param name="optVelocity">优化速度。</param>
         * <param name="directionOpt">如果应该优化方向则为 true。</param>
         * <param name="result">对线性程序结果的引用。</param>
         */
        private int linearProgram2(IList<Line> lines, float radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * 优化方向。 请注意，在这种情况下，优化速度是单位长度。
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* 优化最近点和外圆。 */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* 优化最近点和内圆。 */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0f)
                {
                    /* 结果不满足约束 i。 计算新的最优结果。 */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>求解受直线和圆形约束定义的线性约束影响的二维线性规划。</summary>
         *
         * <param name="lines">定义线性约束的线。</param>
         * <param name="numObstLines">障碍线计数。</param>
         * <param name="beginLine">二维线性程序失败的线。</param>
         * <param name="radius">圆形约束的半径。</param>
         * <param name="result">对线性程序结果的引用。</param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, float radius, ref Vector2 result)
        {
            float distance = 0.0f;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* 结果不满足第 i 行的约束。 */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        float determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (RVOMath.fabs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* 线 i 和线 j 平行。 */
                            if (lines[i].direction * lines[j].direction > 0.0f)
                            {
                                /* i 线和 j 线指向同一方向。 */
                                continue;
                            }
                            else
                            {
                                /* i 线和 j 线指向相反的方向。 */
                                line.point = 0.5f * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, ref result) < projLines.Count)
                    {
                        /*
                         * 原则上这不应该发生。 根据定义，结果已经位于该线性规划的可行区域内。 如果失败，则为浮点误差小，保留当前结果。
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}
