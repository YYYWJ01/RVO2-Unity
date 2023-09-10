/*
 * Circle.cs
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

/*
 * Example file showing a demo with 250 agents initially positioned evenly
 * distributed on a circle attempting to move to the antipodal position on the
 * circle.
 */

#define RVO_OUTPUT_TIME_AND_POSITIONS

using System;
using System.Collections.Generic;

namespace RVO
{
    class Circle
    {
        /* 存储代理的目标。 */
        IList<Vector2> goals;

        Circle()
        {
            goals = new List<Vector2>();
        }

        void setupScenario()
        {
            /* 指定模拟的全局时间步长。 */
            Simulator.Instance.setTimeStep(0.25f);

            /*
             * 为随后添加的代理指定默认参数。
             */
            Simulator.Instance.setAgentDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, new Vector2(0.0f, 0.0f));

            /*
             * 添加代理，指定其起始位置，并将其目标存储在环境的另一侧。
             */
            for (int i = 0; i < 250; ++i)
            {
                Simulator.Instance.addAgent(200.0f *
                    new Vector2((float)Math.Cos(i * 2.0f * Math.PI / 250.0f),
                        (float)Math.Sin(i * 2.0f * Math.PI / 250.0f)));
                goals.Add(-Simulator.Instance.getAgentPosition(i));
            }
        }

        #if RVO_OUTPUT_TIME_AND_POSITIONS
        void updateVisualization()
        {
            /* 输出当前的全局时间。 */
            Console.Write(Simulator.Instance.getGlobalTime());

            /* 输出所有智能体的当前位置。 */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                Console.Write(" {0}", Simulator.Instance.getAgentPosition(i));
            }

            Console.WriteLine();
        }
        #endif

        void setPreferredVelocities()
        {
            /*
             * 将首选速度设置为目标方向上单位幅度（速度）的向量。
             */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                Vector2 goalVector = goals[i] - Simulator.Instance.getAgentPosition(i);

                if (RVOMath.absSq(goalVector) > 1.0f)
                {
                    goalVector = RVOMath.normalize(goalVector);
                }

                Simulator.Instance.setAgentPrefVelocity(i, goalVector);
            }
        }

        bool reachedGoal()
        {
            /* 检查所有代理是否都达到了目标。 */
            for (int i = 0; i < Simulator.Instance.getNumAgents(); ++i)
            {
                if (RVOMath.absSq(Simulator.Instance.getAgentPosition(i) - goals[i]) > Simulator.Instance.getAgentRadius(i) * Simulator.Instance.getAgentRadius(i))
                {
                    return false;
                }
            }
            return true;
        }

        public static void Main(string[] args)
        {
            Circle circle = new Circle();

            /* 设置场景。 */
            circle.setupScenario();

            /* 执行（并操纵）模拟。 */
            do
            {
                #if RVO_OUTPUT_TIME_AND_POSITIONS
                circle.updateVisualization();
                #endif
                circle.setPreferredVelocities();
                Simulator.Instance.doStep();
            }
            while (!circle.reachedGoal());
        }
    }
}
