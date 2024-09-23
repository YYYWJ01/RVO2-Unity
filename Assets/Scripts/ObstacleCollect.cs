using System.Collections;
using System.Collections.Generic;
using RVO;
using UnityEngine;
using Vector2 = RVO.Vector2;

public class ObstacleCollect : MonoBehaviour
{
    void Awake()
    {
        BoxCollider[] boxColliders = GetComponentsInChildren<BoxCollider>();
        for (int i = 0; i < boxColliders.Length; i++)
        {
            // 通过Box障碍物的x位置以及碰撞器的大小&缩放，来计算当前Box障碍物的位置范围（最小 x、z 以及最大 x、z） 。
            float minX = boxColliders[i].transform.position.x -
                         boxColliders[i].size.x*boxColliders[i].transform.lossyScale.x*0.5f;
            float minZ = boxColliders[i].transform.position.z -
                         boxColliders[i].size.z*boxColliders[i].transform.lossyScale.z*0.5f;
            float maxX = boxColliders[i].transform.position.x +
                         boxColliders[i].size.x*boxColliders[i].transform.lossyScale.x*0.5f;
            float maxZ = boxColliders[i].transform.position.z +
                         boxColliders[i].size.z*boxColliders[i].transform.lossyScale.z*0.5f;

            // 根据以上计算结果，构建新的障碍物数据，并添加到RVO障碍物列表内
            IList<Vector2> obstacle = new List<Vector2>();
            obstacle.Add(new Vector2(maxX, maxZ));
            obstacle.Add(new Vector2(minX, maxZ));
            obstacle.Add(new Vector2(minX, minZ));
            obstacle.Add(new Vector2(maxX, minZ));
            Simulator.Instance.addObstacle(obstacle);
        }
    }

    // Update is called once per frame
    void Update()
    {
    }
}