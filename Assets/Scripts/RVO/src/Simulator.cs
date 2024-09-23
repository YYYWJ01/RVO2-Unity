/*
 * Simulator.cs
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
using System.Threading;

namespace RVO
{
    /**
     * <summary>定义模拟。</summary>
     */
    public class Simulator
    {
        /**
         * <summary>定义一个工作者。</summary>
         */
        private class Worker
        {   
            /// <summary>
            /// 手动重置事件
            /// </summary>
            private ManualResetEvent doneEvent_;
            private int end_;
            private int start_;

            /**
             * <summary>构造并初始化工作线程。</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">完成事件.</param>
             */
            internal Worker(int start, int end, ManualResetEvent doneEvent)
            {
                start_ = start;
                end_ = end;
                doneEvent_ = doneEvent;
            }

            internal void config(int start, int end)
            {
                start_ = start;
                end_ = end;
            }

            /**
             * <summary>执行模拟步骤。</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void step(object obj)
            {
                for (int index = start_; index < end_; ++index)
                {
                    Simulator.Instance.agents_[index].computeNeighbors();
                    Simulator.Instance.agents_[index].computeNewVelocity();
                }
                doneEvent_.Set();
            }

            /**
             * <summary>更新每个智能体的二维位置和二维速度。</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void update(object obj)
            {
                for (int index = start_; index < end_; ++index)
                {
                    Simulator.Instance.agents_[index].update();
                }

                doneEvent_.Set();
            }
        }

        internal IDictionary<int, int> agentNo2indexDict_;
        internal IDictionary<int, int> index2agentNoDict_;
        /// <summary>
        /// 代理列表
        /// </summary>
        internal IList<Agent> agents_;
        /// <summary>
        /// 障碍物列表
        /// </summary>
        internal IList<Obstacle> obstacles_;
        /// <summary>
        /// k-D 树
        /// </summary>
        internal KdTree kdTree_;
        /// <summary>
        /// 帧时间
        /// </summary>
        internal float timeStep_;

        private static Simulator instance_ = new Simulator();

        /// <summary>
        /// 默认代理
        /// </summary>
        private Agent defaultAgent_;
        /// <summary>
        /// 完成事件列表
        /// </summary>
        private ManualResetEvent[] doneEvents_;
        /// <summary>
        /// 工作者列表
        /// </summary>
        private Worker[] workers_;
        /// <summary>
        /// 工作者数量
        /// </summary>
        private int numWorkers_;
        /// <summary>
        /// 工作者代理数量
        /// </summary>
        private int workerAgentCount_;
        /// <summary>
        /// 全局时间
        /// </summary>
        private float globalTime_;

        public static Simulator Instance
        {
            get
            {
                return instance_;
            }
        }

        /// <summary>
        /// 删除代理
        /// </summary>
        /// <param name="agentNo"></param>
        public void delAgent(int agentNo)
        {
            agents_[agentNo2indexDict_[agentNo]].needDelete_ = true;
        }

        /// <summary>
        /// 更新删除代理
        /// </summary>
        void updateDeleteAgent()
        {
            bool isDelete = false;
            for (int i = agents_.Count - 1; i >= 0; i--)
            {
                if (agents_[i].needDelete_)
                {
                    agents_.RemoveAt(i);
                    isDelete = true;
                }
            }
            if (isDelete)
                onDelAgent();
        }

        static int s_totalID = 0;
        /**
         * <summary>将具有默认属性的新代理添加到模拟中。 </summary>
         *
         * <returns>代理编号，如果未设置代理默认值，则为 -1。</returns>
         *
         * <param name="position">该代理的二维起始位置。</param>
         */
        public int addAgent(Vector2 position)
        {
            if (defaultAgent_ == null)
            {
                return -1;
            }

            Agent agent = new Agent();
            agent.id_ = s_totalID;
            s_totalID++;
            agent.maxNeighbors_ = defaultAgent_.maxNeighbors_;
            agent.maxSpeed_ = defaultAgent_.maxSpeed_;
            agent.neighborDist_ = defaultAgent_.neighborDist_;
            agent.position_ = position;
            agent.radius_ = defaultAgent_.radius_;
            agent.timeHorizon_ = defaultAgent_.timeHorizon_;
            agent.timeHorizonObst_ = defaultAgent_.timeHorizonObst_;
            agent.velocity_ = defaultAgent_.velocity_;
            agents_.Add(agent);
            onAddAgent();
            return agent.id_;
        }

        void onDelAgent()
        {
            agentNo2indexDict_.Clear();
            index2agentNoDict_.Clear();

            for (int i = 0; i < agents_.Count; i++)
            {
                int agentNo = agents_[i].id_;
                agentNo2indexDict_.Add(agentNo, i);
                index2agentNoDict_.Add(i, agentNo);
            }
        }

        void onAddAgent()
        {
            if (agents_.Count == 0)
                return;

            int index = agents_.Count - 1;
            int agentNo = agents_[index].id_;
            agentNo2indexDict_.Add(agentNo, index);
            index2agentNoDict_.Add(index, agentNo);
        }

        /**
         * <summary>将新代理添加到模拟中。</summary>
         *
         * <returns>代理编号。</returns>
         *
         * <param name="position">这个物体的二维初始位置。</param>
         * <param name="neighborDist">该代理在导航中考虑到的到其他代理的最大距离（中心点到中心点）。 
         * 该数字越大，模拟的运行时间越长。 如果数字太低，模拟将不安全。 必须是非负数。</param>
         * <param name="maxNeighbors">该代理在导航中考虑的其他代理的最大数量。 
         * 该数字越大，模拟的运行时间越长。 如果数字太低，模拟将不安全。</param>
         * <param name="timeHorizon">通过模拟计算出的该代理的速度相对于其他代理来说是安全的最短时间。 
         * 这个数字越大，该代理对其他代理的存在做出响应的速度就越快，但该代理选择其速度的自由度就越小。 
         * 必须是正值。
         * </param>
         * <param name="timeHorizonObst">通过模拟计算出的该代理速度相对于障碍物是安全的最短时间。 
         * 这个数字越大，该智能体对障碍物的存在响应越快，但该智能体选择速度的自由度就越小。 
         * 必须是正值。
         * </param>
         * <param name="maxSpeed">该代理的最大速度。 必须是非负数。</param>
         * <param name="velocity">该代理的初始二维线速度。</param>
         */
        public int addAgent(Vector2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            Agent agent = new Agent();
            agent.id_ = s_totalID;
            s_totalID++;
            agent.maxNeighbors_ = maxNeighbors;
            agent.maxSpeed_ = maxSpeed;
            agent.neighborDist_ = neighborDist;
            agent.position_ = position;
            agent.radius_ = radius;
            agent.timeHorizon_ = timeHorizon;
            agent.timeHorizonObst_ = timeHorizonObst;
            agent.velocity_ = velocity;
            agents_.Add(agent);
            onAddAgent();
            return agent.id_;
        }

        /**
         * <summary>在模拟中增加了一个新的障碍。</summary>
         *
         * <returns>障碍物的第一个顶点的数目，当顶点数目少于两个时为-1。</returns>
         *
         * <param name="vertices">多边形障碍物的顶点列表，按逆时针顺序排列。</param>
         *
         * <remarks>为了添加一个“负面”障碍，例如一个围绕环境的边界多边形，顶点应该按顺时针顺序排列。 </remarks>
         */
        public int addObstacle(IList<Vector2> vertices)
        {
            // 如果 多边形障碍物的顶点列表 数据 < 2，无法形成有效的障碍物，函数返回 -1 表示添加失败。
            if (vertices.Count < 2)
            {
                return -1;
            }
            
            // 通过获取障碍物列表的元素数量来确定新障碍物的编号。
            int obstacleNo = obstacles_.Count;

            // 遍历传入的障碍物顶点列表 vertices
            for (int i = 0; i < vertices.Count; ++i)
            {
                // 根据顶点数据，创建新的障碍物
                Obstacle obstacle = new Obstacle();
                obstacle.point_ = vertices[i];
                
                // 如果不是第一个顶点，就将当前障碍物的 previous_ 属性设置为 obstacles_ 列表中的最后一个障碍物，同时更新最后一个障碍物的 next_ 属性为当前障碍物。
                // 这样彼此连接起来，形成了一个链表。 
                if (i != 0)
                {
                    obstacle.previous_ = obstacles_[obstacles_.Count - 1];
                    obstacle.previous_.next_ = obstacle;
                }

                // 处理最后一个顶点的情况。
                // 如果当前顶点是最后一个顶点，就将当前障碍物的 next_ 属性连接回到起始的障碍物，并将起始障碍物的 previous_ 连接到当前障碍物，以形成闭合的链表环。
                if (i == vertices.Count - 1)
                {
                    obstacle.next_ = obstacles_[obstacleNo];
                    obstacle.next_.previous_ = obstacle;
                }

                // 计算了障碍物的方向向量。它使用 RVOMath.normalize 函数来计算从当前顶点指向下一个顶点的单位向量，表示障碍物的朝向。
                obstacle.direction_ = RVOMath.normalize(vertices[(i == vertices.Count - 1 ? 0 : i + 1)] - vertices[i]);

                /// 计算障碍物是否为凸多边形。
                // 如果顶点数量为2，那么障碍物被认为是凸的。
                // 否则，它使用 RVOMath.leftOf 函数来计算当前顶点、前一个顶点和后一个顶点的位置关系，如果结果大于等于0，表示是凸多边形，否则是凹多边形。
                if (vertices.Count == 2)
                {
                    obstacle.convex_ = true;
                }
                else
                {
                    obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.Count - 1 : i - 1)], vertices[i], vertices[(i == vertices.Count - 1 ? 0 : i + 1)]) >= 0.0f);
                }

                // 每个障碍物被赋予一个唯一的 id_，然后将其添加到 obstacles_ 列表中。
                obstacle.id_ = obstacles_.Count;
                obstacles_.Add(obstacle);
            }

            return obstacleNo;
        }

        /**
         * <summary>清除模拟。</summary>
         */
        public void Clear()
        {
            agents_ = new List<Agent>();
            agentNo2indexDict_ = new Dictionary<int, int>();
            index2agentNoDict_ = new Dictionary<int, int>();
            defaultAgent_ = null;
            kdTree_ = new KdTree();
            obstacles_ = new List<Obstacle>();
            globalTime_ = 0.0f;
            timeStep_ = 0.1f;

            SetNumWorkers(0);
        }

        /**
         * <summary>执行模拟步骤并更新每个代理的二维位置和二维速度。</summary>
         *
         * <returns>模拟步骤后的全局时间。</returns>
         */
        public float doStep()
        {
            // 更新并删除代理
            updateDeleteAgent();

            // 检查是否已经创建了工作线程。如果还没有创建，就会为模拟中的每个工作线程创建一个相关的工作者对象，并为每个工作者对象分配一段代理。
            if (workers_ == null)
            {
                // 初始化工作者线程数组
                workers_ = new Worker[numWorkers_];
                doneEvents_ = new ManualResetEvent[workers_.Length];
                workerAgentCount_ = getNumAgents();

                // 为每个工作者线程创建一个 ManualResetEvent
                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block] = new ManualResetEvent(false);
                    // 创建一个 Worker 对象，并为其分配一段代理
                    workers_[block] = new Worker(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length, doneEvents_[block]);
                }
            }

            // 如果代理数量发生变化，更新工作者线程的分配
            if (workerAgentCount_ != getNumAgents())
            {
                workerAgentCount_ = getNumAgents();
                for (int block = 0; block < workers_.Length; ++block)
                {
                    workers_[block].config(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length);
                }
            }

            // 构建代理树
            kdTree_.buildAgentTree();

            // 遍历模拟中的所有工作者线程。每个工作者线程负责处理一部分代理。
            for (int block = 0; block < workers_.Length; ++block)
            {
                // 重置工作者线程的 ManualResetEvent
                doneEvents_[block].Reset();
                // 将工作者线程的步骤排入线程池，以在后台线程中执行
                ThreadPool.QueueUserWorkItem(workers_[block].step);
            }

            // 等待所有工作者线程完成其工作。它使用 doneEvents_ 数组中的事件来等待线程完成。
            WaitHandle.WaitAll(doneEvents_);

            // 重置工作者线程的 ManualResetEvent
            for (int block = 0; block < workers_.Length; ++block)
            
            {
                doneEvents_[block].Reset();
                // 将工作者线程的更新排入线程池，以在后台线程中执行
                ThreadPool.QueueUserWorkItem(workers_[block].update);
            }

            // 等待所有工作者线程完成其更新
            WaitHandle.WaitAll(doneEvents_);

            // 增加全局时间，通常用于推进模拟的时间步长
            globalTime_ += timeStep_;

            return globalTime_;
        }

        /**
         * <summary>返回指定代理的指定代理邻居。</summary>
         *
         * <returns>相邻代理的编号。</returns>
         *
         * <param name="agentNo">要检索其代理邻居的代理的编号。</param>
         * <param name="neighborNo">要检索的代理邻居的编号。</param>
         */
        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].agentNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>返回指定代理的最大邻居计数。</summary>
         *
         * <returns>代理当前的最大邻居计数。</returns>
         *
         * <param name="agentNo">要检索最大邻居计数的代理的编号。</param>
         */
        public int getAgentMaxNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].maxNeighbors_;
        }

        /**
         * <summary>返回指定代理的最大速度。</summary>
         *
         * <returns>代理当前的最大速度。</returns>
         *
         * <param name="agentNo">要检索最大速度的代理编号。</param>
         */
        public float getAgentMaxSpeed(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].maxSpeed_;
        }

        /**
         * <summary>返回指定代理的最大邻居距离。</summary>
         *
         * <returns>代理当前的最大邻居距离。</returns>
         *
         * <param name="agentNo">要检索最大邻居距离的代理的编号。</param>
         */
        public float getAgentNeighborDist(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].neighborDist_;
        }

        /**
         * <summary>返回计算指定代理的当前速度时考虑的代理邻居的计数。</summary>
         *
         * <returns>计算指定代理的当前速度时考虑的代理邻居的计数。</returns>
         *
         * <param name="agentNo">要检索其代理邻居计数的代理的编号。</param>
         */
        public int getAgentNumAgentNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].agentNeighbors_.Count;
        }

        /**
         * <summary>返回计算指定代理的当前速度时考虑的障碍物邻居的计数。</summary>
         *
         * <returns>计算指定代理的当前速度时考虑的障碍物邻居的数量。</returns>
         *
         * <param name="agentNo">要检索其障碍物邻居计数的代理的编号。</param>
         */
        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].obstacleNeighbors_.Count;
        }

        /**
         * <summary>返回指定代理的指定障碍物邻居。</summary>
         *
         * <returns>相邻障碍物边的第一个顶点的编号。</returns>
         *
         * <param name="agentNo">要检索其障碍邻居的智能体的编号。</param>
         * <param name="neighborNo">要检索的障碍物邻居的编号。</param>
         */
        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].obstacleNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>返回指定代理的 ORCA 约束。</summary>
         *
         * <returns>表示 ORCA 约束的行列表。</returns>
         *
         * <param name="agentNo">要检索 ORCA 约束的代理的编号。</param>
         *
         * <remarks>每条线左侧的半平面是相对于 ORCA 约束的允许速度区域。</remarks>
         */
        public IList<Line> getAgentOrcaLines(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].orcaLines_;
        }

        /**
         * <summary>返回指定代理的二维位置。</summary>
         *
         * <returns>智能体（中心）当前的二维位置。</returns>
         *
         * <param name="agentNo">要检索其二维位置的代理的编号。</param>
         */
        public Vector2 getAgentPosition(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].position_;
        }

        /**
         * <summary>返回指定代理的二维首选速度。</summary>
         *
         * <returns>代理当前的二维首选速度。</returns>
         *
         * <param name="agentNo">要检索其二维首选速度的代理的数量。</param>
         */
        public Vector2 getAgentPrefVelocity(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].prefVelocity_;
        }

        /**
         * <summary>返回指定代理的半径。</summary>
         *
         * <returns>代理的当前半径。</returns>
         *
         * <param name="agentNo">要检索半径的代理编号。</param>
         */
        public float getAgentRadius(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].radius_;
        }

        /**
         * <summary>返回指定代理的时间范围。</summary>
         *
         * <returns>代理的当前时间范围。</returns>
         *
         * <param name="agentNo">要检索时间范围的代理的编号。</param>
         */
        public float getAgentTimeHorizon(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].timeHorizon_;
        }

        /**
         * <summary>返回指定代理的障碍物的时间范围。</summary>
         *
         * <returns>相对于代理障碍的当前时间范围。</returns>
         *
         * <param name="agentNo">要检索其相对于障碍物的时间范围的代理的数量。</param>
         */
        public float getAgentTimeHorizonObst(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].timeHorizonObst_;
        }

        /**
         * <summary>返回指定代理的二维线速度。</summary>
         *
         * <returns>代理当前的二维线速度。</returns>
         *
         * <param name="agentNo">待检索二维线速度的智能体编号。</param>
         */
        public Vector2 getAgentVelocity(int agentNo)
        {
            return agents_[agentNo2indexDict_[agentNo]].velocity_;
        }

        /**
         * <summary>返回模拟的全局时间。</summary>
         *
         * <returns>模拟的当前全局时间（最初为零）。</returns>
         */
        public float getGlobalTime()
        {
            return globalTime_;
        }

        /**
         * <summary>返回模拟中代理的数量。</summary>
         *
         * <returns>模拟中的代理数量。</returns>
         */
        public int getNumAgents()
        {
            return agents_.Count;
        }

        /**
         * <summary>返回模拟中障碍物顶点的计数。</summary>
         *
         * <returns>模拟中障碍物顶点的数量。</returns>
         */
        public int getNumObstacleVertices()
        {
            return obstacles_.Count;
        }

        /**
         * <summary>返回工作者的数量。</summary>
         *
         * <returns>工作者数量。</returns>
         */
        public int GetNumWorkers()
        {
            return numWorkers_;
        }

        /**
         * <summary>返回指定障碍物顶点的二维位置。</summary>
         *
         * <returns>指定障碍物顶点的二维位置。</returns>
         *
         * <param name="vertexNo">要检索的障碍物顶点的数量。</param>
         */
        public Vector2 getObstacleVertex(int vertexNo)
        {
            return obstacles_[vertexNo].point_;
        }

        /**
         * <summary>返回其多边形中指定障碍物顶点之后的障碍物顶点的编号。</summary>
         *
         * <returns>其多边形中指定障碍物顶点之后的障碍物顶点的数量。</returns>
         *
         * <param name="vertexNo">要检索其后继的障碍物顶点的编号。</param>
         */
        public int getNextObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].next_.id_;
        }

        /**
         * <summary>返回其多边形中指定障碍物顶点之前的障碍物顶点的编号。</summary>
         *
         * <returns>其多边形中指定障碍物顶点之前的障碍物顶点的数量。</returns>
         *
         * <param name="vertexNo">要检索其前身的障碍物顶点的编号。</param>
         */
        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].previous_.id_;
        }

        /**
         * <summary>返回模拟的时间步长。</summary>
         *
         * <returns>模拟的当前时间步长。</returns>
         */
        public float getTimeStep()
        {
            return timeStep_;
        }

        /**
         * <summary>处理已添加的障碍物，以便在模拟中考虑到它们。</summary>
         *
         * <remarks>调用此函数后添加到模拟中的障碍不会在模拟中考虑。</remarks>
         */
        public void processObstacles()
        {
            kdTree_.buildObstacleTree();
        }

        /**
         * <summary>执行两个指定点之间相对于障碍物的可见性查询。</summary>
         *
         * <returns>一个布尔值，指定两个点是否相互可见。 当障碍物尚未处理时返回 true。</returns>
         *
         * <param name="point1">询问的第一点。</param>
         * <param name="point2">询问的第二点。</param>
         * <param name="radius">连接两点的线与障碍物之间的最小距离，以使点相互可见（可选）。 必须是非负数。</param>
         */
        public bool queryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            return kdTree_.queryVisibility(point1, point2, radius);
        }

        /// <summary>
        /// 查询附近代理
        /// </summary>
        /// <param name="point">鼠标点击点</param>
        /// <param name="radius">半径</param>
        /// <returns></returns>
        public int queryNearAgent(Vector2 point, float radius)
        {
            if (getNumAgents() == 0)
                return -1;
            return kdTree_.queryNearAgent(point, radius);
        }

        /**
         * <summary>为添加的任何新代理设置默认属性。 </summary>
         *
         * <param name="neighborDist">新代理在导航中考虑到其他代理的默认最大距离（中心点到中心点）。 
         * 这个数字越大，模拟的运行时间就越长。 如果数字太低，模拟将不安全。 必须是非负数。</param>
         * <param name="maxNeighbors">新代理在导航中考虑的其他代理的默认最大数量。 
         * 该数字越大，模拟的运行时间越长。 如果数字太低，模拟将不安全。</param>
         * <param name="timeHorizon">通过模拟计算的新代理速度相对于其他代理来说是安全的默认最短时间。 
         * 这个数字越大，代理对其他代理的存在做出响应的速度就越快，但代理选择其速度的自由度就越小。 必须是正向的。</param>
         * <param name="timeHorizonObst">通过模拟计算的新代理速度相对于障碍物是安全的默认最短时间。 
         * 这个数字越大，智能体对障碍物的存在响应越快，但智能体选择速度的自由度就越小。 必须是正向的。</param>
         * <param name="radius">新代理的默认半径。 必须是非负数。</param>
         * <param name="maxSpeed">新代理的默认最大速度。 必须是非负数。</param>
         * <param name="velocity">新代理的默认初始二维线速度。</param>
         */
        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            if (defaultAgent_ == null)
            {
                defaultAgent_ = new Agent();
            }

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = velocity;
        }

        /**
         * <summary>设置指定代理的最大邻居计数。</summary>
         *
         * <param name="agentNo">要修改最大邻居数的Agent的编号。</param>
         * <param name="maxNeighbors">替换最大邻居数。</param>
         */
        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            agents_[agentNo2indexDict_[agentNo]].maxNeighbors_ = maxNeighbors;
        }

        /**
         * <summary>设置指定代理的最大速度。</summary>
         *
         * <param name="agentNo">需要修改最大速度的代理编号。</param>
         * <param name="maxSpeed">更换最大速度。 必须是非负的。</param>
         */
        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            agents_[agentNo2indexDict_[agentNo]].maxSpeed_ = maxSpeed;
        }

        /**
         * <summary>设置指定代理的最大邻居距离。</summary>
         *
         * <param name="agentNo">要修改最大邻居距离的Agent的编号。</param>
         * <param name="neighborDist">替换最大邻居距离。 必须是非负数。</param>
         */
        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            agents_[agentNo2indexDict_[agentNo]].neighborDist_ = neighborDist;
        }

        /**
         * <summary>设置指定代理的二维位置。</summary>
         *
         * <param name="agentNo">需要修改二维位置的智能体的编号。</param>
         * <param name="position">二维位置的替换。</param>
         */
        public void setAgentPosition(int agentNo, Vector2 position)
        {
            agents_[agentNo2indexDict_[agentNo]].position_ = position;
        }

        /**
         * <summary>设置指定代理的二维首选速度。</summary>
         *
         * <param name="agentNo">要修改二维首选速度的智能体Id。</param>
         * <param name="prefVelocity">二维首选速度的替换。</param>
         */
        public void setAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            agents_[agentNo2indexDict_[agentNo]].prefVelocity_ = prefVelocity;
        }

        /**
         * <summary>设置指定代理的半径。</summary>
         *
         * <param name="agentNo">需要修改半径的代理编号。</param>
         * <param name="radius">替换半径。 必须是非负数。</param>
         */
        public void setAgentRadius(int agentNo, float radius)
        {
            agents_[agentNo2indexDict_[agentNo]].radius_ = radius;
        }

        /**
         * <summary>设置特定代理相对于其他代理的时间范围。</summary>
         *
         * <param name="agentNo">要修改时间范围的代理编号。</param>
         * <param name="timeHorizon">相对于其他代理的替换时间范围。 必须是正值。</param>
         */
        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            agents_[agentNo2indexDict_[agentNo]].timeHorizon_ = timeHorizon;
        }

        /**
         * <summary>设置指定代理相对于障碍物的时间范围。</summary>
         *
         * <param name="agentNo">要修改其相对于障碍物的时间范围的代理的数量。</param>
         * <param name="timeHorizonObst">相对于障碍物的更换时间范围。 必须是正值。</param>
         */
        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            agents_[agentNo2indexDict_[agentNo]].timeHorizonObst_ = timeHorizonObst;
        }

        /**
         * <summary>设置指定代理的二维线速度。</summary>
         *
         * <param name="agentNo">需要修改二维线速度的智能体数量。</param>
         * <param name="velocity">置换二维线速度。</param>
         */
        public void setAgentVelocity(int agentNo, Vector2 velocity)
        {
            agents_[agentNo2indexDict_[agentNo]].velocity_ = velocity;
        }

        /**
         * <summary>设置模拟的全局时间。</summary>
         *
         * <param name="globalTime">模拟的全局时间。</param>
         */
        public void setGlobalTime(float globalTime)
        {
            globalTime_ = globalTime;
        }

        /**
         * <summary>设置工作者数量。</summary>
         *
         * <param name="numWorkers">工作者数量。</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            numWorkers_ = numWorkers;

            if (numWorkers_ <= 0)
            {
                int completionPorts;
                // 获取线程池的最小工作线程数和完成端口线程数
                ThreadPool.GetMinThreads(out numWorkers_, out completionPorts);
            }
            workers_ = null;
            workerAgentCount_ = 0;
        }

        /**
         * <summary>设置模拟的时间步长。</summary>
         *
         * <param name="timeStep">模拟的时间步长。必须是正值。</param>
         */
        public void setTimeStep(float timeStep)
        {
            timeStep_ = timeStep;
        }

        /**
         * <summary>构造并初始化模拟。</summary>
         */
        private Simulator()
        {
            Clear();
        }
    }
}
