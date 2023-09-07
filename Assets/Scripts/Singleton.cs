using UnityEngine;
using System.Collections;
using System;

public abstract class Singleton<T>
{
    private static T _instance;

    private static readonly object _lock = new object();

    public static T Instance
    {
        get
        {
            lock (_lock)
            {
                if (_instance != null)
                    return _instance;

                _instance = (T) Activator.CreateInstance(typeof(T), true);
                (_instance as Singleton<T>).InitInstance();
                return _instance;
            }
        }
    }

    public virtual void InitInstance()
    {
    }
}

/// <summary>
/// 使用InitInstance初始化未唤醒或启动支持多场景，可以在每个场景中添加SingletonBehaviour以支持运行单个场景（使用相同的manager.prefab）
/// </summary>
/// <typeparam name="T"></typeparam>
public abstract class SingletonBehaviour<T> : MonoBehaviour where T : MonoBehaviour
{
    private static T _instance;

    private static object _lock = new object();

    public static T Instance
    {
        get
        {
            if (_applicationIsQuitting)
            {
                Debug.Log("[Singleton] Instance '" + typeof(T) +
                          "' already destroyed on application quit." +
                          " Won't create again - returning null.");
                return null;
            }

            lock (_lock)
            {
                if (_instance != null)
                    return _instance;

                _instance = (T) FindObjectOfType(typeof(T));

                if (FindObjectsOfType(typeof(T)).Length > 1)
                {
                    Debug.LogError("[Singleton] Something went really wrong " +
                                   " - there should never be more than 1 singleton!" +
                                   " Reopening the scene might fix it.");
                    return _instance;
                }

                if (_instance != null)
                    return _instance;

                var singleton = new GameObject();
                _instance = singleton.AddComponent<T>();
                singleton.name = "(singleton) " + typeof(T).ToString();

                Debug.Log("[Singleton] An instance of " + typeof(T) +
                          " is needed in the scene, so '" + singleton +
                          "' was created with DontDestroyOnLoad.");

                return _instance;
            }
        }
    }

    private static bool _applicationIsQuitting = false;

    private void Awake()
    {
        if (Instance == this)
        {
            DontDestroyOnLoad(transform.gameObject);
            InitInstance();
        }
        else
            Destroy(this.gameObject);
    }

    /// <summary>
    /// 当Unity退出时，它会以随机顺序销毁对象。 
    /// 原则上，单例只有在应用程序退出时才会被销毁。 
    /// 如果任何脚本在销毁后调用 Instance，它将创建一个有缺陷的幽灵对象，即使在停止播放应用程序后，该对象也会保留在编辑器场景中。 
    /// 特别糟糕！ 所以，这样做是为了确保我们不会创建那个有缺陷的幽灵对象。
    /// </summary>
    public virtual void OnDestroy()
    {
        if (_instance == this)
            _applicationIsQuitting = true;
    }

    public virtual void InitInstance()
    {
    }
}