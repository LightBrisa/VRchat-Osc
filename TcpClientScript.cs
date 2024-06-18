using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class TcpClientScript : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    private Thread clientThread;

    private void Start()
    {
        clientThread = new Thread(new ThreadStart(ConnectToServer));
        clientThread.IsBackground = true;
        clientThread.Start();
    }

    private void ConnectToServer()
    {
        try
        {
            client = new TcpClient("127.0.0.1", 8888);  // 确保端口号与服务器匹配
            stream = client.GetStream();
            Debug.Log("Connected to server");
            ReceiveData();
        }
        catch (Exception e)
        {
            Debug.LogError("Connection error: " + e.Message);
        }
    }

    private void ReceiveData()
    {
        byte[] data = new byte[256];
        string responseData = string.Empty;

        try
        {
            while (true)
            {
                int bytes = stream.Read(data, 0, data.Length);
                responseData = Encoding.UTF8.GetString(data, 0, bytes);
                Debug.Log("Received data: " + responseData);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Receive error: " + e.Message);
        }
        finally
        {
            stream.Close();
            client.Close();
        }
    }

    private void OnApplicationQuit()
    {
        if (clientThread != null)
        {
            clientThread.Abort();
        }

        if (stream != null)
        {
            stream.Close();
        }

        if (client != null)
        {
            client.Close();
        }
    }
}
