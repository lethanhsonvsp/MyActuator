using System.Runtime.InteropServices;

namespace MyActuator.Services;

public class SocketCan
{
    private const int AF_CAN = 29;
    private const int SOCK_RAW = 3;
    private const int CAN_RAW = 1;

    // ===== SocketCAN options =====
    private const int SOL_CAN_RAW = 101;
    private const int CAN_RAW_RECV_OWN_MSGS = 4;

    private readonly int _socket;

    public bool IsConnected { get; private set; } = false;

    public event Action<CanFrame>? OnFrameReceived;

    // ================= STRUCTS =================

    public struct CanFrame
    {
        public uint Id;
        public byte Dlc;
        public byte[] Data;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct can_frame
    {
        public uint can_id;
        public byte can_dlc;
        public byte __pad1;
        public byte __pad2;
        public byte __pad3;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public byte[] data;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct sockaddr_can
    {
        public ushort can_family;
        public int can_ifindex;
        public uint rx;
        public uint tx;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct ifreq
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public byte[] ifr_name;

        public int ifr_ifindex;

        public ifreq(string name)
        {
            ifr_name = new byte[16];
            var bytes = System.Text.Encoding.ASCII.GetBytes(name);
            Array.Copy(bytes, ifr_name, bytes.Length);
            ifr_ifindex = 0;
        }
    }

    // ================= LINUX SYSCALLS =================

    [DllImport("libc", SetLastError = true)]
    static extern int socket(int domain, int type, int protocol);

    [DllImport("libc", SetLastError = true)]
    static extern int bind(int sockfd, ref sockaddr_can addr, int addrlen);

    [DllImport("libc", SetLastError = true)]
    static extern int ioctl(int fd, uint request, ref ifreq ifr);

    [DllImport("libc", SetLastError = true)]
    static extern int read(int fd, byte[] buffer, int count);

    [DllImport("libc", SetLastError = true)]
    static extern int write(int fd, byte[] buffer, int count);

    [DllImport("libc", SetLastError = true)]
    static extern int setsockopt(
        int sockfd,
        int level,
        int optname,
        ref int optval,
        uint optlen
    );

    private const uint SIOCGIFINDEX = 0x8933;

    // ================= CONSTRUCTOR =================

    public SocketCan(string iface)
    {
        try
        {
            _socket = socket(AF_CAN, SOCK_RAW, CAN_RAW);
            if (_socket < 0)
            {
                IsConnected = false;
                return;
            }

            // ============================================================
            // ✅ ENABLE RX OF OWN TX FRAMES (TX LOOPBACK)
            // ============================================================
            int enable = 1;
            setsockopt(
                _socket,
                SOL_CAN_RAW,
                CAN_RAW_RECV_OWN_MSGS,
                ref enable,
                (uint)sizeof(int)
            );

            var ifr = new ifreq(iface);
            if (ioctl(_socket, SIOCGIFINDEX, ref ifr) < 0)
            {
                IsConnected = false;
                return;
            }

            var addr = new sockaddr_can
            {
                can_family = AF_CAN,
                can_ifindex = ifr.ifr_ifindex
            };

            if (bind(_socket, ref addr, Marshal.SizeOf<sockaddr_can>()) < 0)
            {
                IsConnected = false;
                return;
            }

            IsConnected = true;
        }
        catch
        {
            IsConnected = false;
        }
    }

    // ================= SEND =================

    /// <summary>
    /// Gửi 1 frame CAN (ID 11-bit, DLC 0–8)
    /// </summary>
    public void Send(uint id, byte[] data)
    {
        if (!IsConnected)
            return;

        if (data.Length > 8)
            throw new ArgumentException("CAN Data must be <= 8 bytes");

        var frame = new can_frame
        {
            can_id = id & 0x7FF, // 11-bit standard ID
            can_dlc = (byte)data.Length,
            data = new byte[8]
        };

        Array.Copy(data, frame.data, data.Length);
        // data frame is always 8 bytes in size
        // Console.WriteLine(
        //     $"➡ Sending CAN Frame: ID=0x{frame.can_id:X3} " +
        //     $"DLC={frame.can_dlc} Data={BitConverter.ToString(data)}"
        // );
        int size = Marshal.SizeOf<can_frame>();
        byte[] buffer = StructToBytes(frame);

        int result = write(_socket, buffer, size);
        if (result < 0)
            Console.WriteLine("⚠ CAN write() failed");
    }

    // ================= RECEIVE LOOP =================

    public void StartReading(CancellationToken token)
    {
        int size = Marshal.SizeOf<can_frame>();
        byte[] buffer = new byte[size];

        while (!token.IsCancellationRequested)
        {
            int n = read(_socket, buffer, size);
            if (n <= 0)
            {
                IsConnected = false;
                return;
            }

            var frame = ByteArrayToStruct<can_frame>(buffer);

            OnFrameReceived?.Invoke(new CanFrame
            {
                Id = frame.can_id & 0x1FFFFFFF,
                Dlc = frame.can_dlc,
                Data = frame.data
            });
        }
    }

    // ================= MARSHAL HELPERS =================

    private static T ByteArrayToStruct<T>(byte[] bytes)
    {
        GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        T obj = Marshal.PtrToStructure<T>(handle.AddrOfPinnedObject())!;
        handle.Free();
        return obj;
    }

    private static byte[] StructToBytes(object obj)
    {
        int size = Marshal.SizeOf(obj);
        byte[] arr = new byte[size];

        GCHandle handle = GCHandle.Alloc(arr, GCHandleType.Pinned);
        Marshal.StructureToPtr(obj, handle.AddrOfPinnedObject(), false);
        handle.Free();

        return arr;
    }
}
