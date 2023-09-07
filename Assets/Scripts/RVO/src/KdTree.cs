/*
 * KdTree.cs
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

using System.Collections.Generic;
using System;

namespace RVO
{
    /**
     * <summary>为模拟中的代理和静态障碍物定义 k-D 树。</summary>
     */
    internal class KdTree
    {
        /**
         * <summary>定义代理 k-D 树的节点。</summary>
         */
        private struct AgentTreeNode
        {
            internal int begin_;
            internal int end_;
            internal int left_;
            internal int right_;
            internal float maxX_;
            internal float maxY_;
            internal float minX_;
            internal float minY_;
        }

        /**
         * <summary>定义一对标量值。</summary>
         */
        private struct FloatPair
        {
            private float a_;
            private float b_;

            /**
             * <summary>构造并初始化一对标量值。</summary>
             *
             * <param name="a">第一个标量值。</returns>
             * <param name="b">第二个标量值。</returns>
             */
            internal FloatPair(float a, float b)
            {
                a_ = a;
                b_ = b;
            }

            /**
             * <summary>如果第一对标量值小于第二对标量值，则返回 true。</summary>
             *
             * <returns>如果第一对标量值小于第二对标量值，则为 True。</returns>
             *
             * <param name="pair1">第一对标量值。</param>
             * <param name="pair2">第二对标量值。</param>
             */
            public static bool operator <(FloatPair pair1, FloatPair pair2)
            {
                return pair1.a_ < pair2.a_ || !(pair2.a_ < pair1.a_) && pair1.b_ < pair2.b_;
            }

            /**
             * <summary>如果第一对标量值小于或等于第二对标量值，则返回 true。</summary>
             *
             * <returns>如果第一对标量值小于或等于第二对标量值，则为 True。</returns>
             *
             * <param name="pair1">第一对标量值。</param>
             * <param name="pair2">第二对标量值。</param>
             */
            public static bool operator <=(FloatPair pair1, FloatPair pair2)
            {
                return (pair1.a_ == pair2.a_ && pair1.b_ == pair2.b_) || pair1 < pair2;
            }

            /**
             * <summary>如果第一对标量值大于第二对标量值，则返回 true。</summary>
             *
             * <returns>如果第一对标量值大于第二对标量值，则为 True。</returns>
             *
             * <param name="pair1">第一对标量值。</param>
             * <param name="pair2">第二对标量值。</param>
             */
            public static bool operator >(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 <= pair2);
            }

            /**
             * <summary>如果第一对标量值大于或等于第二对标量值，则返回 true。 </summary>
             *
             * <returns>如果第一对标量值大于或等于第二对标量值，则为 True。</returns>
             *
             * <param name="pair1">第一对标量值。</param>
             * <param name="pair2">第二对标量值。</param>
             */
            public static bool operator >=(FloatPair pair1, FloatPair pair2)
            {
                return !(pair1 < pair2);
            }
        }

        /**
         * <summary>定义障碍物 k-D 树的节点。</summary>
         */
        private class ObstacleTreeNode
        {
            /// <summary>
            /// 障碍物
            /// </summary>
            internal Obstacle obstacle_;
            /// <summary>
            /// 障碍物 k-D 树的左节点
            /// </summary>
            internal ObstacleTreeNode left_;
            /// <summary>
            /// 障碍物 k-D 树的右节点
            /// </summary>
            internal ObstacleTreeNode right_;
        };

        /**
         * <summary>代理 k-D 树叶的最大尺寸。</summary>
         */
        private const int MAX_LEAF_SIZE = 10;

        private Agent[] agents_;
        private AgentTreeNode[] agentTree_;
        private ObstacleTreeNode obstacleTree_;

        /**
         * <summary>Builds an agent k-D tree.</summary>
         */
        internal void buildAgentTree()
        {
            if (agents_ == null || agents_.Length != Simulator.Instance.agents_.Count)
            {
                agents_ = new Agent[Simulator.Instance.agents_.Count];

                for (int i = 0; i < agents_.Length; ++i)
                {
                    agents_[i] = Simulator.Instance.agents_[i];
                }

                agentTree_ = new AgentTreeNode[2 * agents_.Length];

                for (int i = 0; i < agentTree_.Length; ++i)
                {
                    agentTree_[i] = new AgentTreeNode();
                }
            }

            if (agents_.Length != 0)
            {
                buildAgentTreeRecursive(0, agents_.Length, 0);
            }
        }

        /**
         * <summary>构建障碍物 k-D 树。</summary>
         */
        internal void buildObstacleTree()
        {
            // 创建障碍树节点
            obstacleTree_ = new ObstacleTreeNode();

            // 创建障碍物列表
            IList<Obstacle> obstacles = new List<Obstacle>(Simulator.Instance.obstacles_.Count);

            // 将 模拟的障碍物 加入到 障碍物 列表内
            for (int i = 0; i < Simulator.Instance.obstacles_.Count; ++i)
            {
                obstacles.Add(Simulator.Instance.obstacles_[i]);
            }

            // 构建障碍物 k-D 树
            obstacleTree_ = buildObstacleTreeRecursive(obstacles);
        }

        /**
         * <summary>计算指定代理的代理邻居。</summary>
         *
         * <param name="agent">要计算代理邻居的代理。</param>
         * <param name="rangeSq">代理周围的平方范围。</param>
         */
        internal void computeAgentNeighbors(Agent agent, ref float rangeSq)
        {
            queryAgentTreeRecursive(agent, ref rangeSq, 0);
        }

        /**
         * <summary>Computes the obstacle neighbors of the specified agent.
         * </summary>
         *
         * <param name="agent">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         */
        internal void computeObstacleNeighbors(Agent agent, float rangeSq)
        {
            queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
        }

        /**
         * <summary>Queries the visibility between two points within a specified
         * radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         */
        internal bool queryVisibility(Vector2 q1, Vector2 q2, float radius)
        {
            return queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
        }

        internal int queryNearAgent(Vector2 point, float radius)
        {
            float rangeSq = float.MaxValue;
            int agentNo = -1;
            queryAgentTreeRecursive(point, ref rangeSq, ref agentNo, 0);
            if (rangeSq < radius*radius)
                return agentNo;
            return -1;
        }

        /**
         * <summary>Recursive method for building an agent k-D tree.</summary>
         *
         * <param name="begin">The beginning agent k-D tree node node index.
         * </param>
         * <param name="end">The ending agent k-D tree node index.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void buildAgentTreeRecursive(int begin, int end, int node)
        {
            agentTree_[node].begin_ = begin;
            agentTree_[node].end_ = end;
            agentTree_[node].minX_ = agentTree_[node].maxX_ = agents_[begin].position_.x_;
            agentTree_[node].minY_ = agentTree_[node].maxY_ = agents_[begin].position_.y_;

            for (int i = begin + 1; i < end; ++i)
            {
                agentTree_[node].maxX_ = Math.Max(agentTree_[node].maxX_, agents_[i].position_.x_);
                agentTree_[node].minX_ = Math.Min(agentTree_[node].minX_, agents_[i].position_.x_);
                agentTree_[node].maxY_ = Math.Max(agentTree_[node].maxY_, agents_[i].position_.y_);
                agentTree_[node].minY_ = Math.Min(agentTree_[node].minY_, agents_[i].position_.y_);
            }

            if (end - begin > MAX_LEAF_SIZE)
            {
                /* No leaf node. */
                bool isVertical = agentTree_[node].maxX_ - agentTree_[node].minX_ > agentTree_[node].maxY_ - agentTree_[node].minY_;
                float splitValue = 0.5f * (isVertical ? agentTree_[node].maxX_ + agentTree_[node].minX_ : agentTree_[node].maxY_ + agentTree_[node].minY_);

                int left = begin;
                int right = end;

                while (left < right)
                {
                    while (left < right && (isVertical ? agents_[left].position_.x_ : agents_[left].position_.y_) < splitValue)
                    {
                        ++left;
                    }

                    while (right > left && (isVertical ? agents_[right - 1].position_.x_ : agents_[right - 1].position_.y_) >= splitValue)
                    {
                        --right;
                    }

                    if (left < right)
                    {
                        Agent tempAgent = agents_[left];
                        agents_[left] = agents_[right - 1];
                        agents_[right - 1] = tempAgent;
                        ++left;
                        --right;
                    }
                }

                int leftSize = left - begin;

                if (leftSize == 0)
                {
                    ++leftSize;
                    ++left;
                    ++right;
                }

                agentTree_[node].left_ = node + 1;
                agentTree_[node].right_ = node + 2 * leftSize;

                buildAgentTreeRecursive(begin, left, agentTree_[node].left_);
                buildAgentTreeRecursive(left, end, agentTree_[node].right_);
            }
        }

        /**
         * <summary>构建障碍物 k-D 树的递归方法。</summary> 
         *
         * PS：这个树结构用于在路径规划中进行碰撞检测和避障操作。
         * 
         * <returns>障碍物 k-D 树节点。</returns>
         *
         * <param name="obstacles">障碍清单。</param>
         */
        private ObstacleTreeNode buildObstacleTreeRecursive(IList<Obstacle> obstacles)
        {
            // 检查传入的障碍物列表是否为空。如果列表为空，意味着没有障碍物可用于构建 k-D 树，所以直接返回 null，表示没有树节点。
            if (obstacles.Count == 0)
            {
                return null;
            }

            // 创建障碍物 k-D 树的一个节点
            ObstacleTreeNode node = new ObstacleTreeNode();

            // 最优分割点
            int optimalSplit = 0;
            // 左子树的最小障碍物数量(初始值设为障碍物列表的总数)
            int minLeft = obstacles.Count;
            // 右子树的最小障碍物数量(初始值设为障碍物列表的总数)
            int minRight = obstacles.Count;
            
            // 遍历障碍物列表中的每一个障碍物，找出最优分割点以及左子树与右子树的最小障碍物数量
            for (int i = 0; i < obstacles.Count; ++i)
            {
                // 记录位于当前障碍物左侧和右侧的障碍物数量
                int leftSize = 0;
                int rightSize = 0;

                // 获取当前的障碍物
                Obstacle obstacleI1 = obstacles[i];
                // 获取 当前障碍物 的 下一个障碍物
                Obstacle obstacleI2 = obstacleI1.next_;

                /* 计算最佳分裂节点。 */
                for (int j = 0; j < obstacles.Count; ++j)
                {   
                    // 如果外层索引与内层索引一致，则认为是同一个障碍物，直接跳过后续逻辑
                    if (i == j)
                    {
                        continue;
                    }

                    // 获取当前选中的其他障碍物
                    Obstacle obstacleJ1 = obstacles[j];
                    // 获取 当前选中其他障碍物 的 下一个障碍物
                    Obstacle obstacleJ2 = obstacleJ1.next_;

                    // 检测 obstacleJ1 是否位于 obstacleI1 与 obstacleI2 线段的左侧(j1LeftOfI > 0 左侧，j1LeftOfI < 0 右侧,j1LeftOfI = 0 位于线段上)
                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    // 检测 obstacleJ2 是否位于 obstacleI1 与 obstacleI2 线段的左侧
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);
                    
                    // 如果 j1LeftOfI 和 j2LeftOfI 都为非负值，则判定 左侧叶节点 +1
                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        ++leftSize;
                    }
                    // 如果 j1LeftOfI 和 j2LeftOfI 都为负值，则判定 右侧叶节点 +1
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        ++rightSize;
                    }
                    // 否则 左侧叶节点和右侧叶节点都 +1
                    else
                    {
                        ++leftSize;
                        ++rightSize;
                    }

                    // 用于比较当前障碍物组合的左侧和右侧数量是否大于等于已知的最小数量（minLeft 和 minRight）。如果是，就跳出内层 for 循环，因为已经找到了一个更好的分割点。
                    if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) >= new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                    {
                        break; 
                    }
                }
                
                /// 更新最小左侧和右侧数量以及最佳分割点。
                // 如果当前障碍物组合的左侧和右侧数量都小于已知的最小数量，那么将 minLeft 和 minRight 更新为当前左右数量，并将 optimalSplit 更新为当前索引 i，表示找到了一个更好的分割点。
                if (new FloatPair(Math.Max(leftSize, rightSize), Math.Min(leftSize, rightSize)) < new FloatPair(Math.Max(minLeft, minRight), Math.Min(minLeft, minRight)))
                {
                    minLeft = leftSize;
                    minRight = rightSize;
                    optimalSplit = i;
                }
            }

            {
                /* 构建分裂节点。 */
                // 左子树的障碍物列表
                IList<Obstacle> leftObstacles = new List<Obstacle>(minLeft);
                for (int n = 0; n < minLeft; ++n)
                {
                    leftObstacles.Add(null);
                }

                // 右子树的障碍物列表
                IList<Obstacle> rightObstacles = new List<Obstacle>(minRight);
                for (int n = 0; n < minRight; ++n)
                {
                    rightObstacles.Add(null);
                }

                // 左子树计数器
                int leftCounter = 0;
                // 右子树计数器
                int rightCounter = 0;
                // 最优分割点
                int i = optimalSplit;

                // 最优分割点的障碍物
                Obstacle obstacleI1 = obstacles[i];
                // 最优分割点的障碍物 的 下一个障碍物
                Obstacle obstacleI2 = obstacleI1.next_;

                // 将障碍物分为左子树和右子树。在内循环中，同样检查是否当前障碍物与自身相同，如果是则跳过。
                for (int j = 0; j < obstacles.Count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    
                    // 获取当前选中的其他障碍物
                    Obstacle obstacleJ1 = obstacles[j];
                    // 获取 当前选中其他障碍物 的 下一个障碍物
                    Obstacle obstacleJ2 = obstacleJ1.next_;
                    
                    // 检测 obstacleJ1 是否位于 obstacleI1 与 obstacleI2 线段的左侧(j1LeftOfI > 0 左侧，j1LeftOfI < 0 右侧,j1LeftOfI = 0 位于线段上)
                    float j1LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ1.point_);
                    // 检测 obstacleJ2 是否位于 obstacleI1 与 obstacleI2 线段的左侧
                    float j2LeftOfI = RVOMath.leftOf(obstacleI1.point_, obstacleI2.point_, obstacleJ2.point_);

                    // 如果 j1LeftOfI 和 j2LeftOfI 都为非负值，则记录为 左子树障碍物节点
                    if (j1LeftOfI >= -RVOMath.RVO_EPSILON && j2LeftOfI >= -RVOMath.RVO_EPSILON)
                    {
                        leftObstacles[leftCounter++] = obstacles[j];
                    }
                    // 如果 j1LeftOfI 和 j2LeftOfI 都为负值，则记录为 右子树障碍物节点
                    else if (j1LeftOfI <= RVOMath.RVO_EPSILON && j2LeftOfI <= RVOMath.RVO_EPSILON)
                    {
                        rightObstacles[rightCounter++] = obstacles[j];
                    }
                    else
                    {
                        /* 分割障碍物 j. */
                        // 如果障碍物既不全在左侧也不全在右侧，那么就需要对它进行分割。

                        // 通过求解两个向量的叉积来找到相交点。
                        float t = RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleI1.point_) / RVOMath.det(obstacleI2.point_ - obstacleI1.point_, obstacleJ1.point_ - obstacleJ2.point_);

                        Vector2 splitPoint = obstacleJ1.point_ + t * (obstacleJ2.point_ - obstacleJ1.point_);

                        // 在找到分割点后，创建了一个新的障碍物 newObstacle，并将其属性设置为合适的值。然后，将这个新的障碍物添加到障碍物列表中，并更新相邻障碍物的连接关系。
                        Obstacle newObstacle = new Obstacle();
                        newObstacle.point_ = splitPoint;
                        newObstacle.previous_ = obstacleJ1;
                        newObstacle.next_ = obstacleJ2;
                        newObstacle.convex_ = true;
                        newObstacle.direction_ = obstacleJ1.direction_;

                        newObstacle.id_ = Simulator.Instance.obstacles_.Count;

                        Simulator.Instance.obstacles_.Add(newObstacle);

                        obstacleJ1.next_ = newObstacle;
                        obstacleJ2.previous_ = newObstacle;

                        // 根据 j1LeftOfI 的值，将当前障碍物添加到左子树或右子树，并更新计数器。
                        if (j1LeftOfI > 0.0f)
                        {
                            leftObstacles[leftCounter++] = obstacleJ1;
                            rightObstacles[rightCounter++] = newObstacle;
                        }
                        else
                        {
                            rightObstacles[rightCounter++] = obstacleJ1;
                            leftObstacles[leftCounter++] = newObstacle;
                        }
                    }
                }

                // 用于构建分裂节点，将当前节点 node 设置为 obstacleI1，并递归地构建左子树和右子树，分别传入 leftObstacles 和 rightObstacles。
                node.obstacle_ = obstacleI1;
                node.left_ = buildObstacleTreeRecursive(leftObstacles);
                node.right_ = buildObstacleTreeRecursive(rightObstacles);

                return node;
            }
        }

        private void queryAgentTreeRecursive(Vector2 position, ref float rangeSq, ref int agentNo, int node)
        {
            if (agentTree_[node].end_ - agentTree_[node].begin_ <= MAX_LEAF_SIZE)
            {
                for (int i = agentTree_[node].begin_; i < agentTree_[node].end_; ++i)
                {
                    float distSq = RVOMath.absSq(position - agents_[i].position_);
                    if (distSq < rangeSq)
                    {
                        rangeSq = distSq;
                        agentNo = agents_[i].id_;
                    }
                }
            }
            else
            {
                float distSqLeft = RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].left_].minX_ - position.x_)) + RVOMath.sqr(Math.Max(0.0f, position.x_ - agentTree_[agentTree_[node].left_].maxX_)) + RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].left_].minY_ - position.y_)) + RVOMath.sqr(Math.Max(0.0f, position.y_ - agentTree_[agentTree_[node].left_].maxY_));
                float distSqRight = RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].right_].minX_ - position.x_)) + RVOMath.sqr(Math.Max(0.0f, position.x_ - agentTree_[agentTree_[node].right_].maxX_)) + RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].right_].minY_ - position.y_)) + RVOMath.sqr(Math.Max(0.0f, position.y_ - agentTree_[agentTree_[node].right_].maxY_));

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree_[node].left_);

                        if (distSqRight < rangeSq)
                        {
                            queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree_[node].right_);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree_[node].right_);

                        if (distSqLeft < rangeSq)
                        {
                            queryAgentTreeRecursive(position, ref rangeSq, ref agentNo, agentTree_[node].left_);
                        }
                    }
                }

            }
        }

        /**
         * <summary>Recursive method for computing the agent neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which agent neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current agent k-D tree node index.</param>
         */
        private void queryAgentTreeRecursive(Agent agent, ref float rangeSq, int node)
        {
            if (agentTree_[node].end_ - agentTree_[node].begin_ <= MAX_LEAF_SIZE)
            {
                for (int i = agentTree_[node].begin_; i < agentTree_[node].end_; ++i)
                {
                    agent.insertAgentNeighbor(agents_[i], ref rangeSq);
                }
            }
            else
            {
                float distSqLeft = RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].left_].minX_ - agent.position_.x_)) + RVOMath.sqr(Math.Max(0.0f, agent.position_.x_ - agentTree_[agentTree_[node].left_].maxX_)) + RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].left_].minY_ - agent.position_.y_)) + RVOMath.sqr(Math.Max(0.0f, agent.position_.y_ - agentTree_[agentTree_[node].left_].maxY_));
                float distSqRight = RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].right_].minX_ - agent.position_.x_)) + RVOMath.sqr(Math.Max(0.0f, agent.position_.x_ - agentTree_[agentTree_[node].right_].maxX_)) + RVOMath.sqr(Math.Max(0.0f, agentTree_[agentTree_[node].right_].minY_ - agent.position_.y_)) + RVOMath.sqr(Math.Max(0.0f, agent.position_.y_ - agentTree_[agentTree_[node].right_].maxY_));

                if (distSqLeft < distSqRight)
                {
                    if (distSqLeft < rangeSq)
                    {
                        queryAgentTreeRecursive(agent, ref rangeSq, agentTree_[node].left_);

                        if (distSqRight < rangeSq)
                        {
                            queryAgentTreeRecursive(agent, ref rangeSq, agentTree_[node].right_);
                        }
                    }
                }
                else
                {
                    if (distSqRight < rangeSq)
                    {
                        queryAgentTreeRecursive(agent, ref rangeSq, agentTree_[node].right_);

                        if (distSqLeft < rangeSq)
                        {
                            queryAgentTreeRecursive(agent, ref rangeSq, agentTree_[node].left_);
                        }
                    }
                }

            }
        }

        /**
         * <summary>Recursive method for computing the obstacle neighbors of the
         * specified agent.</summary>
         *
         * <param name="agent">The agent for which obstacle neighbors are to be
         * computed.</param>
         * <param name="rangeSq">The squared range around the agent.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private void queryObstacleTreeRecursive(Agent agent, float rangeSq, ObstacleTreeNode node)
        {
            if (node != null)
            {
                Obstacle obstacle1 = node.obstacle_;
                Obstacle obstacle2 = obstacle1.next_;

                float agentLeftOfLine = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, agent.position_);

                queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.left_ : node.right_);

                float distSqLine = RVOMath.sqr(agentLeftOfLine) / RVOMath.absSq(obstacle2.point_ - obstacle1.point_);

                if (distSqLine < rangeSq)
                {
                    if (agentLeftOfLine < 0.0f)
                    {
                        /*
                         * Try obstacle at this node only if agent is on right side of
                         * obstacle (and can see obstacle).
                         */
                        agent.insertObstacleNeighbor(node.obstacle_, rangeSq);
                    }

                    /* Try other side of line. */
                    queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0f ? node.right_ : node.left_);
                }
            }
        }

        /**
         * <summary>Recursive method for querying the visibility between two
         * points within a specified radius.</summary>
         *
         * <returns>True if q1 and q2 are mutually visible within the radius;
         * false otherwise.</returns>
         *
         * <param name="q1">The first point between which visibility is to be
         * tested.</param>
         * <param name="q2">The second point between which visibility is to be
         * tested.</param>
         * <param name="radius">The radius within which visibility is to be
         * tested.</param>
         * <param name="node">The current obstacle k-D node.</param>
         */
        private bool queryVisibilityRecursive(Vector2 q1, Vector2 q2, float radius, ObstacleTreeNode node)
        {
            if (node == null)
            {
                return true;
            }

            Obstacle obstacle1 = node.obstacle_;
            Obstacle obstacle2 = obstacle1.next_;

            float q1LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q1);
            float q2LeftOfI = RVOMath.leftOf(obstacle1.point_, obstacle2.point_, q2);
            float invLengthI = 1.0f / RVOMath.absSq(obstacle2.point_ - obstacle1.point_);

            if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.left_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.right_));
            }

            if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
            {
                return queryVisibilityRecursive(q1, q2, radius, node.right_) && ((RVOMath.sqr(q1LeftOfI) * invLengthI >= RVOMath.sqr(radius) && RVOMath.sqr(q2LeftOfI) * invLengthI >= RVOMath.sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node.left_));
            }

            if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f)
            {
                /* One can see through obstacle from left to right. */
                return queryVisibilityRecursive(q1, q2, radius, node.left_) && queryVisibilityRecursive(q1, q2, radius, node.right_);
            }

            float point1LeftOfQ = RVOMath.leftOf(q1, q2, obstacle1.point_);
            float point2LeftOfQ = RVOMath.leftOf(q1, q2, obstacle2.point_);
            float invLengthQ = 1.0f / RVOMath.absSq(q2 - q1);

            return point1LeftOfQ * point2LeftOfQ >= 0.0f && RVOMath.sqr(point1LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && RVOMath.sqr(point2LeftOfQ) * invLengthQ > RVOMath.sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node.left_) && queryVisibilityRecursive(q1, q2, radius, node.right_);
        }
    }
}
