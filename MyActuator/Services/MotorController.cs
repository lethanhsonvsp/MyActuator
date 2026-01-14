namespace MyActuator.Services;


// ==================== MOTOR CONTROLLER ====================
/// <summary>
/// Class điều khiển motor MYACTUATOR qua CAN Bus
/// Hỗ trợ các lệnh: Position, Velocity, Torque control
/// </summary>
public class MotorController
{
    private readonly SocketCan _can;
    private readonly byte _motorId;
    private CancellationTokenSource? _readCts;

    // Motor status
    public int Temperature { get; private set; }
    public float TorqueCurrent { get; private set; }
    public int Speed { get; private set; }
    public int Angle { get; private set; }
    public int EncoderPosition { get; private set; }
    public ushort ErrorState { get; private set; }
    public byte Voltage { get; private set; }

    public bool IsConnected => _can.IsConnected;

    // Events
    public event Action<MotorStatus>? OnStatusUpdate;
    public event Action<string>? OnError;

    public MotorController(string canInterface, byte motorId)
    {
        if (motorId < 1 || motorId > 32)
            throw new ArgumentException("Motor ID must be 1-32");

        _motorId = motorId;
        _can = new SocketCan(canInterface);

        if (_can.IsConnected)
        {
            _can.OnFrameReceived += HandleCanFrame;
            StartReading();
        }
    }

    private void StartReading()
    {
        _readCts = new CancellationTokenSource();
        Task.Run(() => _can.StartReading(_readCts.Token));
    }

    public void Stop()
    {
        _readCts?.Cancel();
    }

    // ================= CAN FRAME HANDLER =================
    private void HandleCanFrame(SocketCan.CanFrame frame)
    {
        // Check if frame is for this motor (Reply ID = 0x240 + ID)
        uint expectedReplyId = (uint)(0x240 + _motorId);
        if (frame.Id != expectedReplyId)
            return;

        byte cmd = frame.Data[0];

        switch (cmd)
        {
            case 0x9C: // Read Motor Status 2
                ParseStatus2(frame.Data);
                break;
            case 0x9A: // Read Motor Status 1
                ParseStatus1(frame.Data);
                break;
            case 0xA1: // Torque Control Reply
            case 0xA2: // Velocity Control Reply
            case 0xA4: // Position Control Reply
                ParseControlReply(frame.Data);
                break;
        }
    }

    private void ParseStatus2(byte[] data)
    {
        Temperature = (sbyte)data[1];
        TorqueCurrent = BitConverter.ToInt16(data, 2) * 0.01f;
        Speed = BitConverter.ToInt16(data, 4);
        Angle = BitConverter.ToInt16(data, 6);

        OnStatusUpdate?.Invoke(GetStatus());
    }

    private void ParseStatus1(byte[] data)
    {
        Temperature = (sbyte)data[1];
        Voltage = (byte)(BitConverter.ToUInt16(data, 4) * 0.1f);
        ErrorState = BitConverter.ToUInt16(data, 6);

        if (ErrorState != 0)
            OnError?.Invoke($"Motor Error: 0x{ErrorState:X4}");
    }

    private void ParseControlReply(byte[] data)
    {
        Temperature = (sbyte)data[1];
        TorqueCurrent = BitConverter.ToInt16(data, 2) * 0.01f;
        Speed = BitConverter.ToInt16(data, 4);
        Angle = BitConverter.ToInt16(data, 6);

        OnStatusUpdate?.Invoke(GetStatus());
    }

    // ================= CONTROL COMMANDS =================

    /// <summary>
    /// Đọc trạng thái motor
    /// </summary>
    public void ReadMotorStatus()
    {
        SendCommand(0x9C, new byte[7]);
    }

    /// <summary>
    /// Tắt motor (0x80)
    /// </summary>
    public void ShutdownMotor()
    {
        SendCommand(0x80, new byte[7]);
    }

    /// <summary>
    /// Dừng motor (0x81)
    /// </summary>
    public void StopMotor()
    {
        SendCommand(0x81, new byte[7]);
    }

    /// <summary>
    /// Điều khiển moment (torque) - Command 0xA1
    /// </summary>
    /// <param name="torqueCurrent">Dòng điện moment (A), range: -10A to +10A</param>
    public void SetTorque(float torqueCurrent)
    {
        // Convert A to 0.01A/LSB
        short iqControl = (short)(torqueCurrent * 100);

        byte[] data = new byte[7];
        data[0] = 0xA1;
        Array.Copy(BitConverter.GetBytes(iqControl), 0, data, 3, 2);

        SendCommand(0xA1, data);
    }

    /// <summary>
    /// Điều khiển tốc độ - Command 0xA2
    /// </summary>
    /// <param name="speedDps">Tốc độ (degrees/second), unit: 0.01dps/LSB</param>
    /// <param name="maxTorque">Moment tối đa (% rated current), 0-255</param>
    public void SetVelocity(int speedDps, byte maxTorque = 0)
    {
        // Convert dps to 0.01dps/LSB
        int speedControl = speedDps * 100;

        byte[] data = new byte[7];
        data[0] = maxTorque;
        Array.Copy(BitConverter.GetBytes(speedControl), 0, data, 3, 4);

        SendCommand(0xA2, data);
    }

    /// <summary>
    /// Điều khiển vị trí tuyệt đối - Command 0xA4
    /// </summary>
    /// <param name="angleDegree">Góc mục tiêu (degrees), unit: 0.01degree/LSB</param>
    /// <param name="maxSpeed">Tốc độ tối đa (dps), unit: 1dps/LSB</param>
    public void SetPosition(float angleDegree, ushort maxSpeed = 500)
    {
        // Convert degree to 0.01degree/LSB
        int angleControl = (int)(angleDegree * 100);

        byte[] data = new byte[7];
        Array.Copy(BitConverter.GetBytes(maxSpeed), 0, data, 1, 2);
        Array.Copy(BitConverter.GetBytes(angleControl), 0, data, 3, 4);

        SendCommand(0xA4, data);
    }

    /// <summary>
    /// Điều khiển vị trí tăng dần - Command 0xA8
    /// </summary>
    /// <param name="incrementDegree">Góc tăng dần (degrees)</param>
    /// <param name="maxSpeed">Tốc độ tối đa (dps)</param>
    public void SetIncrementalPosition(float incrementDegree, ushort maxSpeed = 500)
    {
        int angleControl = (int)(incrementDegree * 100);

        byte[] data = new byte[7];
        Array.Copy(BitConverter.GetBytes(maxSpeed), 0, data, 1, 2);
        Array.Copy(BitConverter.GetBytes(angleControl), 0, data, 3, 4);

        SendCommand(0xA8, data);
    }

    /// <summary>
    /// Đọc vị trí encoder multi-turn - Command 0x60
    /// </summary>
    public void ReadEncoderPosition()
    {
        SendCommand(0x60, new byte[7]);
    }

    /// <summary>
    /// Đọi PID parameters - Command 0x30
    /// </summary>
    /// <param name="index">Index: 0x01=Current KP, 0x04=Speed KP, 0x07=Position KP</param>
    public void ReadPID(byte index)
    {
        byte[] data = new byte[7];
        data[0] = index;
        SendCommand(0x30, data);
    }

    /// <summary>
    /// Ghi PID parameters vào RAM - Command 0x31
    /// </summary>
    public void WritePIDToRAM(byte index, float value)
    {
        byte[] data = new byte[7];
        data[0] = index;
        Array.Copy(BitConverter.GetBytes(value), 0, data, 3, 4);
        SendCommand(0x31, data);
    }

    /// <summary>
    /// Reset hệ thống - Command 0x76
    /// </summary>
    public void ResetSystem()
    {
        SendCommand(0x76, new byte[7]);
    }

    // ================= HELPER METHODS =================

    private void SendCommand(byte command, byte[] parameters)
    {
        if (!_can.IsConnected)
        {
            OnError?.Invoke("CAN not connected");
            return;
        }

        byte[] data = new byte[8];
        data[0] = command;
        Array.Copy(parameters, 0, data, 1, Math.Min(parameters.Length, 7));

        uint canId = (uint)(0x140 + _motorId);
        _can.Send(canId, data);
    }

    public MotorStatus GetStatus()
    {
        return new MotorStatus
        {
            Temperature = Temperature,
            TorqueCurrent = TorqueCurrent,
            Speed = Speed,
            Angle = Angle,
            EncoderPosition = EncoderPosition,
            ErrorState = ErrorState,
            Voltage = Voltage,
            IsConnected = IsConnected
        };
    }
}

// ==================== MOTOR STATUS ====================
public struct MotorStatus
{
    public int Temperature { get; set; }
    public float TorqueCurrent { get; set; }
    public int Speed { get; set; }
    public int Angle { get; set; }
    public int EncoderPosition { get; set; }
    public ushort ErrorState { get; set; }
    public byte Voltage { get; set; }
    public bool IsConnected { get; set; }
}

// ==================== SERVICE FOR BLAZOR ====================
/// <summary>
/// Service để điều khiển motor trong Blazor
/// Sử dụng event để trigger UI refresh
/// </summary>
public class MotorControlService
{
    private MotorController? _motor;
    private Timer? _statusTimer;

    // Properties - dùng auto-property cho Blazor
    public string ConnectionStatus { get; private set; } = "Disconnected";
    public int Temperature { get; private set; }
    public float TorqueCurrent { get; private set; }
    public int Speed { get; private set; }
    public int Angle { get; private set; }
    public string ErrorMessage { get; private set; } = "";
    public bool IsConnected => _motor?.IsConnected ?? false;

    // Control inputs
    public float TargetTorque { get; set; }
    public int TargetSpeed { get; set; } = 100;
    public float TargetAngle { get; set; }
    public ushort MaxSpeed { get; set; } = 500;

    public List<string> Logs { get; } = new();

    // Event để trigger UI refresh trong Blazor
    public event Action? OnStateChanged;

    // ================= CONNECTION =================

    public async Task ConnectAsync(string canInterface, byte motorId)
    {
        await Task.Run(() =>
        {
            try
            {
                _motor = new MotorController(canInterface, motorId);

                if (_motor.IsConnected)
                {
                    _motor.OnStatusUpdate += HandleStatusUpdate;
                    _motor.OnError += HandleError;

                    ConnectionStatus = $"Connected to Motor ID {motorId}";
                    AddLog($"✓ Connected to {canInterface}, Motor ID {motorId}");

                    // Start periodic status reading (every 100ms)
                    _statusTimer = new Timer(_ =>
                    {
                        _motor.ReadMotorStatus();
                    }, null, 0, 100);
                }
                else
                {
                    ConnectionStatus = "Connection Failed";
                    AddLog($"✗ Failed to connect to {canInterface}");
                }
            }
            catch (Exception ex)
            {
                ConnectionStatus = "Error";
                AddLog($"✗ Error: {ex.Message}");
            }

            NotifyStateChanged();
        });
    }

    public void Disconnect()
    {
        _statusTimer?.Dispose();
        _motor?.Stop();
        ConnectionStatus = "Disconnected";
        AddLog("Disconnected");
        NotifyStateChanged();
    }

    // ================= CONTROL COMMANDS =================

    public void SendTorqueCommand()
    {
        _motor?.SetTorque(TargetTorque);
        AddLog($"→ Set Torque: {TargetTorque:F2} A");
        NotifyStateChanged();
    }

    public void SendVelocityCommand()
    {
        _motor?.SetVelocity(TargetSpeed);
        AddLog($"→ Set Velocity: {TargetSpeed} dps");
        NotifyStateChanged();
    }

    public void SendPositionCommand()
    {
        _motor?.SetPosition(TargetAngle, MaxSpeed);
        AddLog($"→ Set Position: {TargetAngle:F2}° @ {MaxSpeed} dps");
        NotifyStateChanged();
    }

    public void StopMotor()
    {
        _motor?.StopMotor();
        AddLog("→ Motor Stopped");
        NotifyStateChanged();
    }

    public void ShutdownMotor()
    {
        _motor?.ShutdownMotor();
        AddLog("→ Motor Shutdown");
        NotifyStateChanged();
    }

    public void ResetMotor()
    {
        _motor?.ResetSystem();
        AddLog("→ System Reset");
        NotifyStateChanged();
    }

    // ================= EVENT HANDLERS =================

    private void HandleStatusUpdate(MotorStatus status)
    {
        Temperature = status.Temperature;
        TorqueCurrent = status.TorqueCurrent;
        Speed = status.Speed;
        Angle = status.Angle;
        NotifyStateChanged();
    }

    private void HandleError(string error)
    {
        ErrorMessage = error;
        AddLog($"⚠ {error}");
        NotifyStateChanged();
    }

    private void AddLog(string message)
    {
        string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
        Logs.Add($"[{timestamp}] {message}");

        // Keep only last 100 logs
        while (Logs.Count > 100)
            Logs.RemoveAt(0);
    }

    private void NotifyStateChanged() => OnStateChanged?.Invoke();
}