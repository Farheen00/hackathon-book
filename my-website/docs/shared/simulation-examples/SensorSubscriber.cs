using ROS2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SensorSubscriber : MonoBehaviour
{
    ROSConnection ros;
    string topicName = "/gazebo_sensor_data";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
        // Subscribe to the topic
        ros.Subscribe<Float32Msg>(topicName, OnSensorDataReceived);
    }

    void OnSensorDataReceived(Float32Msg sensorMsg)
    {
        // Process the received sensor data
        Debug.Log("Received sensor data: " + sensorMsg.data);

        // Update Unity object based on sensor data
        UpdateObjectPosition(sensorMsg.data);
    }

    void UpdateObjectPosition(float sensorValue)
    {
        // Example: Move object based on sensor value
        transform.position = new Vector3(transform.position.x, sensorValue, transform.position.z);
    }
}