using ROS2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SensorPublisher : MonoBehaviour
{
    ROSConnection ros;
    string topicName = "/unity_sensor_data";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
    }

    void FixedUpdate()
    {
        // Publish sensor data to ROS
        var sensorData = new Float32Msg();
        sensorData.data = transform.position.y;
        ros.Publish<Float32Msg>(topicName, sensorData);
    }
}