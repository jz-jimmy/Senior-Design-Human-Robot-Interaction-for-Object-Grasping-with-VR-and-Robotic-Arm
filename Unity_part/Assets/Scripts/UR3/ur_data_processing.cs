/****************************************************************************
MIT License
Copyright(c) 2021 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************
Author   : Roman Parak
Email    : Roman.Parak @outlook.com
Github   : https://github.com/rparak
File Name: ur_data_processing.cs
****************************************************************************/

// System
using System;
using System.Diagnostics;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
// Unity 
using UnityEngine;
using Debug = UnityEngine.Debug;
// Missing using directive
using System.IO;
using System.IO.Ports;


public class ur_data_processing : MonoBehaviour
{
    public static class GlobalVariables_Main_Control
    {
        public static bool connect, disconnect;
    }

    public static class UR_Stream_Data
    {
        // IP Port Number and IP Address
        public static string ip_address;
        //  Real-time (Read Only)
        public const ushort port_number = 30013; // 30001? original 30013 30004 might work
        // Comunication Speed (ms)
        public static int time_step;
        // Joint Space:
        //  Orientation {J1 .. J6} (rad)
        public static double[] J_Orientation = new double[6];
        // Cartesian Space:
        //  Position {X, Y, Z} (mm)
        public static double[] C_Position = new double[3];
        //  Orientation {Euler Angles} (rad):
        public static double[] C_Orientation = new double[3];
        // Class thread information (is alive or not)
        public static bool is_alive = false;

        // Data recording
        public static List<byte[]> recordedPackets = new List<byte[]>();
        public static bool isRecording = false;
        public static long lastRecordTime = 0;
        public static long lastSaveTime = 0;
        public const int RECORD_INTERVAL = 1000; // Record every 1000ms
        public const int AUTO_SAVE_INTERVAL = 10000; // Auto-save every 10 seconds
    }
    public static class UR_Control_Data
    {
        // IP Port Number and IP Address
        public static string ip_address;
        //  Real-time (Read/Write)
        public const ushort port_number = 30003;
        // Comunication Speed (ms)
        public static int time_step;
        // Control Parameters UR3/UR3e:
        public static string aux_command_str;
        public static byte[] command;
        public static bool[] button_pressed = new bool[12];
        public static bool joystick_button_pressed;
        // Class thread information (is alive or not)
        public static bool is_alive = false;
    }


     public class SerialHandler
    {
        private SerialPort serialPort;
        private string portName;
        private int baudRate;

        public SerialHandler(string portName, int baudRate)
        {
            this.portName = portName;
            this.baudRate = baudRate;
        }

        public bool IsOpen
        {
            get { return serialPort != null && serialPort.IsOpen; }
        }

        public void Open()
        {
            try
            {
                serialPort = new SerialPort(portName, baudRate);
                serialPort.Open();
                Debug.Log("串口已打开");
            }
            catch (Exception ex)
            {
                Debug.LogError("打开串口失败: " + ex.Message);
            }
        }

        public void Close()
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.Close();
                Debug.Log("串口已关闭");
            }
        }

        public void WriteData(string data)
        {
            if (serialPort != null && serialPort.IsOpen)
            {
                serialPort.WriteLine(data);
                Debug.Log("数据发送成功");
            }
        }
    }

    // Class Stream / Control {Universal Robots TCP/IP}
    private UR_Stream ur_stream_robot;
    private UR_Control ur_ctrl_robot;
    // Other variables
    private int main_ur3_state = 0;
    private int aux_counter_pressed_btn = 0;
    private SerialHandler serialHandler;

    // Start is called before the first frame update
    void Start()
    {
        // Initialization {TCP/IP Universal Robots}
        //  Read Data:
        UR_Stream_Data.ip_address = "192.168.1.120";
        //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
        UR_Stream_Data.time_step = 1;
        //  Write Data:
        UR_Control_Data.ip_address = "192.168.1.120";
        //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
        UR_Control_Data.time_step = 8;

        // Initialization Stream {Universal Robots TCP/IP}
        ur_stream_robot = new UR_Stream();
        // Start Control {Universal Robots TCP/IP}
        ur_ctrl_robot = new UR_Control();

        // Automatically start recording when the application starts
        // ur_stream_robot.StartRecording();
        serialHandler = new SerialHandler("COM3", 115200);  // 初始化串口操作
        serialHandler.Open();
        // serialHandler.WriteData("S\r\n");
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        switch (main_ur3_state)
        {
            case 0:
                {
                    // ------------------------ Wait State {Disconnect State} ------------------------//

                    if (GlobalVariables_Main_Control.connect == true)
                    {
                        // Start Stream {Universal Robots TCP/IP}
                        ur_stream_robot.Start();
                        // Start Control {Universal Robots TCP/IP}
                        ur_ctrl_robot.Start();

                        // go to connect state
                        main_ur3_state = 1;
                    }
                }
                break;
            case 1:
                {
                    // ------------------------ Data Processing State {Connect State} ------------------------//

                    for (int i = 0; i < UR_Control_Data.button_pressed.Length; i++)
                    {
                        // check the pressed button in joystick control mode
                        if (UR_Control_Data.button_pressed[i] == true)
                        {
                            aux_counter_pressed_btn++;
                        }
                    }

                    // at least one button pressed
                    if (aux_counter_pressed_btn > 0)
                    {
                        // start move -> speed control
                        UR_Control_Data.joystick_button_pressed = true;
                    }
                    else
                    {
                        // stop move -> speed control
                        UR_Control_Data.joystick_button_pressed = false;
                    }

                    // null auxiliary variable
                    aux_counter_pressed_btn = 0;

                    // Periodically auto-save recorded data
                    if (UR_Stream_Data.isRecording)
                    {
                        long currentTime = DateTimeOffset.Now.ToUnixTimeMilliseconds();
                        if (currentTime - UR_Stream_Data.lastSaveTime >= UR_Stream_Data.AUTO_SAVE_INTERVAL && 
                            UR_Stream_Data.recordedPackets.Count > 0)
                        {
                            ur_stream_robot.SaveRecordedData();
                            UR_Stream_Data.lastSaveTime = currentTime;
                        }
                    }

                    if (GlobalVariables_Main_Control.disconnect == true)
                    {
                        // Save any remaining data if recording
                        if (UR_Stream_Data.isRecording && UR_Stream_Data.recordedPackets.Count > 0)
                        {
                            ur_stream_robot.StopRecording();
                        }

                        // Stop threading block {TCP/Ip -> read data}
                        if (UR_Stream_Data.is_alive == true)
                        {
                            ur_stream_robot.Stop();
                        }
                        // Stop threading block {TCP/Ip  -> write data}
                        if (UR_Control_Data.is_alive == true)
                        {
                            ur_ctrl_robot.Stop();
                        }
                        if (UR_Stream_Data.is_alive == false && UR_Control_Data.is_alive == false)
                        {
                            // go to initialization state {wait state -> disconnect state}
                            main_ur3_state = 0;
                        }
                    }
                }
                break;
        }
    }

    void OnApplicationQuit()
    {
        try
        {
            // Destroy Stream {Universal Robots TCP/IP}
            ur_stream_robot.Destroy();
            // Destroy Control {Universal Robots TCP/IP}
            ur_ctrl_robot.Destroy();

            Destroy(this);
        }
        catch (Exception e)
        {
           Debug.LogException(e);
        }
    }

    class UR_Stream
    {
        // Initialization of Class variables
        //  Thread
        private Thread robot_thread = null;
        private bool exit_thread = false;
        //  TCP/IP Communication
        private TcpClient tcp_client = new TcpClient();
        private NetworkStream network_stream = null;
        //  Packet Buffer (Read)
        private byte[] packet = new byte[1116];
        //  Main state machine
        private int state_id = 0;

        // Offset:
        //  Size of first packet in bytes (Integer)
        private const byte first_packet_size = 4;
        //  Size of other packets in bytes (Double)
        private const byte offset = 8;

        // Total message length in bytes
        private static List<UInt32> msg_length_list = new List<UInt32>();
        private static UInt32 total_msg_length = 0;

        public void UR_Stream_Thread()
        {
            try
            {
                if (tcp_client.Connected == false)
                {
                    // Connect to controller -> if the controller is disconnected
                    tcp_client.Connect(UR_Stream_Data.ip_address, UR_Stream_Data.port_number);
                }

                // Initialization TCP/IP Communication (Stream)
                network_stream = tcp_client.GetStream();

                // Initialization timer
                var t = new Stopwatch();

                // Clear message length list before starting
                msg_length_list.Clear();
                total_msg_length = 0;

                while (exit_thread == false)
                {
                    switch (state_id)
                    {
                        case 0:
                            {
                                // Getting the total message length from several runs of reading data
                                if (network_stream.Read(packet, 0, packet.Length) != 0)
                                {
                                    if (msg_length_list.Count == 10)
                                    {
                                        msg_length_list.Sort();
                                        total_msg_length = msg_length_list[msg_length_list.Count - 1];
                                        state_id = 1;
                                        Debug.Log("Successfully established connection with the robot. Starting to receive data.");
                                    }
                                    else
                                    {
                                        msg_length_list.Add(BitConverter.ToUInt32(packet, first_packet_size - 4));
                                    }
                                }
                            }
                            break;

                        case 1:
                        {
                            // Get the data from the robot
                            if (network_stream.Read(packet, 0, packet.Length) != 0)
                            {
                                if (BitConverter.ToUInt32(packet, first_packet_size - 4) == total_msg_length)
                                {
                                    // t_{0}: Timer start.
                                    t.Start();
        
                                    // Reverses the order of elements in a one-dimensional array or part of an array.
                                    Array.Reverse(packet);
        
                                    // Note:
                                    //  For more information on values 32... 37, etc., see the UR Client Interface document.
                                    // Read Joint Values in radians
                                    UR_Stream_Data.J_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset));
                                    UR_Stream_Data.J_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset));
                                    UR_Stream_Data.J_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset));
                                    UR_Stream_Data.J_Orientation[3] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset));
                                    UR_Stream_Data.J_Orientation[4] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset));
                                    UR_Stream_Data.J_Orientation[5] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset));
                                    // Read Cartesian (Positon) Values in metres
                                    UR_Stream_Data.C_Position[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (56 * offset));
                                    UR_Stream_Data.C_Position[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (57 * offset));
                                    UR_Stream_Data.C_Position[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (58 * offset));
                                    // Read Cartesian (Orientation) Values in metres 
                                    UR_Stream_Data.C_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (59 * offset));
                                    UR_Stream_Data.C_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (60 * offset));
                                    UR_Stream_Data.C_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (61 * offset));

                                    // Record data every 1000ms if recording is enabled
                                    if (UR_Stream_Data.isRecording)
                                    {
                                        long currentTime = DateTimeOffset.Now.ToUnixTimeMilliseconds();
                                        if (currentTime - UR_Stream_Data.lastRecordTime >= UR_Stream_Data.RECORD_INTERVAL)
                                        {
                                            byte[] packetCopy = new byte[packet.Length];
                                            Array.Copy(packet, packetCopy, packet.Length);
                                            UR_Stream_Data.recordedPackets.Add(packetCopy);
                                            UR_Stream_Data.lastRecordTime = currentTime;
                                            Debug.Log($"Recorded data at {currentTime}ms, total records: {UR_Stream_Data.recordedPackets.Count}, Joint1: {UR_Stream_Data.J_Orientation[0]}, X: {UR_Stream_Data.C_Position[0]}");
                                        }
                                    }
        
                                    // t_{1}: Timer stop.
                                    t.Stop();
        
                                    // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds
                                    if (t.ElapsedMilliseconds < UR_Stream_Data.time_step)
                                    {
                                        Thread.Sleep(UR_Stream_Data.time_step - (int)t.ElapsedMilliseconds);
                                    }
        
                                    // Reset (Restart) timer.
                                    t.Restart();
                                }
                            }
                        }
                        break;
                    }
                }
            }
            catch (SocketException e)
            {
                Debug.LogException(e);
            }
        }

        public void Start()
        {
            // Start thread
            exit_thread = false;
            // Start a thread and listen to incoming messages
            robot_thread = new Thread(new ThreadStart(UR_Stream_Thread));
            robot_thread.IsBackground = true;
            robot_thread.Start();
            // Thread is active
            UR_Stream_Data.is_alive = true;
            Debug.Log("UR_Stream thread started");
        }
        public void Stop()
        {
            exit_thread = true;
            // Stop a thread
            Thread.Sleep(100);
            UR_Stream_Data.is_alive = robot_thread.IsAlive;
            robot_thread.Abort();
        }
        public void Destroy()
        {
            if (tcp_client.Connected == true)
            {
                // Disconnect communication
                network_stream.Dispose();
                tcp_client.Close();
            }
            Thread.Sleep(100);
        }

        // Add methods to control recording
        public void StartRecording()
        {
            UR_Stream_Data.isRecording = true;
            UR_Stream_Data.recordedPackets.Clear();
            UR_Stream_Data.lastRecordTime = DateTimeOffset.Now.ToUnixTimeMilliseconds();
            UR_Stream_Data.lastSaveTime = UR_Stream_Data.lastRecordTime;
            Debug.Log("Recording started");
        }

        public void StopRecording()
        {
            UR_Stream_Data.isRecording = false;
            SaveRecordedData();
            Debug.Log("Recording stopped and data saved");
        }

        public void SaveRecordedData()
        {
            try
            {
                // If no data recorded, return
                if (UR_Stream_Data.recordedPackets.Count == 0)
                {
                    Debug.LogWarning("No data to save. Make sure the robot is connected and data is being recorded.");
                    return;
                }

                // Create directory if it doesn't exist
                string directory = "E:\\Study\\ECE445\\Data";
                Debug.Log($"Attempting to save data to directory: {directory}");
                
                try
                {
                    if (!System.IO.Directory.Exists(directory))
                    {
                        Debug.Log($"Directory does not exist, creating: {directory}");
                        System.IO.Directory.CreateDirectory(directory);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"Failed to create directory: {e.Message}");
                    // Try to save to a more accessible location
                    directory = Application.dataPath;
                    Debug.Log($"Falling back to saving in application data path: {directory}");
                }

                // Generate unique filename with timestamp
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string filename = $"robot_data_{timestamp}.csv";
                string filePath = System.IO.Path.Combine(directory, filename);
                Debug.Log($"Attempting to save data to file: {filePath}");

                // Try to create a test file to check write permissions
                string testFile = System.IO.Path.Combine(directory, "test_write.tmp");
                try
                {
                    System.IO.File.WriteAllText(testFile, "test");
                    System.IO.File.Delete(testFile);
                }
                catch (Exception e)
                {
                    Debug.LogError($"No write permission to directory: {directory}. Error: {e.Message}");
                    // Try to save to a more accessible location
                    directory = Application.temporaryCachePath;
                    filePath = System.IO.Path.Combine(directory, filename);
                    Debug.Log($"Falling back to temporary cache path: {directory}");
                }

                using (System.IO.StreamWriter writer = new System.IO.StreamWriter(filePath))
                {
                    writer.WriteLine("Timestamp,J1,J2,J3,J4,J5,J6,X,Y,Z,RX,RY,RZ");
                    
                    int processedCount = 0;
                    foreach (byte[] packet in UR_Stream_Data.recordedPackets)
                    {
                        try
                        {
                            // Make a copy to avoid modifying the original
                            byte[] packetCopy = new byte[packet.Length];
                            Array.Copy(packet, packetCopy, packet.Length);
                            
                            // Reverses the order of elements in a one-dimensional array or part of an array.
                            Array.Reverse(packetCopy);

                            // Extract data from packet
                            double[] joints = new double[6];
                            double[] position = new double[3];
                            double[] orientation = new double[3];

                            // Read Joint Values in radians
                            joints[0] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (32 * offset));
                            joints[1] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (33 * offset));
                            joints[2] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (34 * offset));
                            joints[3] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (35 * offset));
                            joints[4] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (36 * offset));
                            joints[5] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (37 * offset));

                            // Read Cartesian (Position) Values in metres
                            position[0] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (56 * offset));
                            position[1] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (57 * offset));
                            position[2] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (58 * offset));

                            // Read Cartesian (Orientation) Values in radians
                            orientation[0] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (59 * offset));
                            orientation[1] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (60 * offset));
                            orientation[2] = BitConverter.ToDouble(packetCopy, packetCopy.Length - first_packet_size - (61 * offset));

                            // Write data to file
                            writer.WriteLine($"{DateTime.Now.AddMilliseconds(processedCount * UR_Stream_Data.RECORD_INTERVAL):yyyy-MM-dd HH:mm:ss.fff}," +
                                $"{joints[0]:F6},{joints[1]:F6},{joints[2]:F6},{joints[3]:F6},{joints[4]:F6},{joints[5]:F6}," +
                                $"{position[0]:F6},{position[1]:F6},{position[2]:F6}," +
                                $"{orientation[0]:F6},{orientation[1]:F6},{orientation[2]:F6}");
                            
                            processedCount++;
                        }
                        catch (Exception e)
                        {
                            Debug.LogError($"Error processing packet: {e.Message}");
                            // Continue with the next packet
                            continue;
                        }
                    }
                }
                
                Debug.Log($"Successfully saved {UR_Stream_Data.recordedPackets.Count} records to: {filePath}");

                // Don't clear the packets here to allow multiple saves
            }
            catch (Exception e)
            {
                Debug.LogError($"Error saving data: {e.Message}\nStack trace: {e.StackTrace}");
                
                // Try one last desperate attempt to save the data
                try {
                    string emergencyPath = System.IO.Path.Combine(Application.temporaryCachePath, "emergency_robot_data.txt");
                    System.IO.File.WriteAllText(emergencyPath, $"Error occurred but saved data count: {UR_Stream_Data.recordedPackets.Count}");
                    Debug.Log($"Emergency data indicator saved to: {emergencyPath}");
                } catch (Exception ex) {
                    Debug.LogError($"Even emergency save failed: {ex.Message}");
                }
            }
        }

        public List<byte[]> GetRecordedData()
        {
            return UR_Stream_Data.recordedPackets;
        }
    }

    class UR_Control
    {
        // Initialization of Class variables
        //  Thread
        private Thread robot_thread = null;
        private bool exit_thread = false;
        //  TCP/IP Communication
        private TcpClient tcp_client = new TcpClient();
        private NetworkStream network_stream = null;

        public void UR_Control_Thread()
        {
            try
            {
                if (tcp_client.Connected != true)
                {
                    // Connect to controller -> if the controller is disconnected
                    tcp_client.Connect(UR_Control_Data.ip_address, UR_Control_Data.port_number);

                }

                // Initialization TCP/IP Communication (Stream)
                network_stream = tcp_client.GetStream();

                // Initialization timer
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    // t_{0}: Timer start.
                    t.Start();

                    // Note:
                    //  For more information about commands, see the URScript Programming Language document 

                    if (UR_Control_Data.joystick_button_pressed == true)
                    {
                        // Send command (byte) -> speed control of the robot (X,Y,Z and EA{RX, RY, RZ})
                        network_stream.Write(UR_Control_Data.command, 0, UR_Control_Data.command.Length);
                    }

                    // t_{1}: Timer stop.
                    t.Stop();

                    // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds
                    if (t.ElapsedMilliseconds < UR_Control_Data.time_step)
                    {
                        Thread.Sleep(UR_Control_Data.time_step - (int)t.ElapsedMilliseconds);
                    }

                    // Reset (Restart) timer.
                    t.Restart();
                }
            }
            catch (SocketException e)
            {
                Debug.LogException(e);
            }
        }

        public void Start()
        {
            // Start thread
            exit_thread = false;
            // Start a thread and listen to incoming messages
            robot_thread = new Thread(new ThreadStart(UR_Control_Thread));
            robot_thread.IsBackground = true;
            robot_thread.Start();
            // Thread is active
            UR_Control_Data.is_alive = true;
        }
        public void Stop()
        {
            exit_thread = true;
            // Stop a thread
            Thread.Sleep(100);
            UR_Control_Data.is_alive = robot_thread.IsAlive;
            robot_thread.Abort();
        }
        public void Destroy()
        {
            if (tcp_client.Connected == true)
            {
                // Disconnect communication
                network_stream.Dispose();
                tcp_client.Close();
            }
            Thread.Sleep(100);
        }
    }

    // 公开函数用于切换串口状态
    public void ToggleSerialPort()
    {
        if (serialHandler != null)
        {
            if (serialHandler.IsOpen)
            {
                // 发送关闭命令
                serialHandler.WriteData("\r");
                Debug.Log("发送关闭命令");
                serialHandler.Close();
                Debug.Log("串口已手动关闭");
            }
            else
            {
                serialHandler.Open();
                serialHandler.WriteData("S\r\n");
                Debug.Log("串口已手动打开并发送S命令");
            }
        }
        else
        {
            Debug.LogError("serialHandler未初始化！");
            // 尝试重新初始化
            serialHandler = new SerialHandler("COM3", 115200);
            serialHandler.Open();
            serialHandler.WriteData("S\r\n");
        }
    }
    
    // 公开函数用于发送夹持命令
    public void SendGripCommand()
    {
        if (serialHandler != null && serialHandler.IsOpen)
        {
            serialHandler.WriteData("S\r\n");
            Debug.Log("发送夹持命令");
        }
        else
        {
            Debug.LogError("串口未打开或未初始化，无法发送夹持命令！");
            // 尝试重新初始化并打开
            serialHandler = new SerialHandler("COM3", 115200);
            serialHandler.Open();
            serialHandler.WriteData("S\r\n");
            Debug.Log("已重新打开串口并发送夹持命令");
        }
    }
    
    // 公开函数用于发送放开命令
    public void SendReleaseCommand()
    {
        if (serialHandler != null && serialHandler.IsOpen)
        {
            serialHandler.WriteData("R\r\n");
            Debug.Log("发送放开命令");
        }
        else
        {
            Debug.LogError("串口未打开或未初始化，无法发送放开命令！");
        }
    }
}
