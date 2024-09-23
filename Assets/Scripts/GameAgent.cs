using System;
using System.Collections;
using System.Collections.Generic;
using RVO;
using UnityEngine;
using Random = System.Random;
using Vector2 = RVO.Vector2;

/// <summary>
/// 代理对象
/// </summary>
public class GameAgent : MonoBehaviour
{
    [HideInInspector] public int sid = -1;

    /** 随机数生成器。 */
    private Random m_random = new Random();
    // Use this for initialization
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (sid >= 0)
        {
            Vector2 pos = Simulator.Instance.getAgentPosition(sid);
            Vector2 vel = Simulator.Instance.getAgentPrefVelocity(sid);
            transform.position = new Vector3(pos.x(), transform.position.y, pos.y());
            if (Math.Abs(vel.x()) > 0.01f && Math.Abs(vel.y()) > 0.01f)
                transform.forward = new Vector3(vel.x(), 0, vel.y()).normalized;
        }
        
        // 检测是否按下鼠标右键
        if (!Input.GetMouseButton(1))
        {
            Simulator.Instance.setAgentPrefVelocity(sid, new Vector2(0, 0));
            return;
        }

        // 从代理位置指向鼠标位置的向量
        Vector2 goalVector = GameMainManager.Instance.mousePosition - Simulator.Instance.getAgentPosition(sid);
        // 检查向量是否太短
        if (RVOMath.absSq(goalVector) > 1.0f)
        {
            // 将 goalVector 标准化，即将它的长度调整为1，但方向保持不变。这是为了确保代理以合适的速度朝向目标位置移动，而不会以非常快的速度移动。
            goalVector = RVOMath.normalize(goalVector);
        }

        Simulator.Instance.setAgentPrefVelocity(sid, goalVector);

        /* 稍微扰动一下以避免由于完美对称而导致死锁。 */
        // 生成 0 ~ 360 度的随机角度
        float angle = (float) m_random.NextDouble()*2.0f*(float) Math.PI;
        // 生成缩放到一个 0 ~ 0.0001 之间的距离范围
        float dist = (float) m_random.NextDouble()*0.0001f;

        Simulator.Instance.setAgentPrefVelocity(sid, Simulator.Instance.getAgentPrefVelocity(sid) +
                                                     dist*
                                                     new Vector2((float) Math.Cos(angle), (float) Math.Sin(angle)));
    }
}