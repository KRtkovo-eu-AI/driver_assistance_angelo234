using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace PCDClient;

internal static class Program
{
    private const int FloatsPerPoint = 4;
    private const int BytesPerFloat = 4;

    private static async Task Main()
    {
        Console.OutputEncoding = Encoding.UTF8;
        Console.WriteLine("BeamNG LiDAR TCP → PCD collector\n");

        (string host, int port) = PromptForEndpoint();

        using var client = new TcpClient();
        try
        {
            Console.WriteLine($"Connecting to {host}:{port} …");
            await client.ConnectAsync(host, port);
        }
        catch (SocketException ex)
        {
            Console.WriteLine($"Failed to connect to the server ({ex.SocketErrorCode}): {ex.Message}");
            return;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Failed to connect: {ex.Message}");
            return;
        }

        using NetworkStream stream = client.GetStream();
        stream.ReadTimeout = 1000; // milliseconds
        client.NoDelay = true;

        using var aggregatedData = new MemoryStream();
        long totalPoints = 0;
        long totalPayloadBytes = 0;
        long frames = 0;
        bool userCancelled = false;
        bool streamClosed = false;

        Console.WriteLine("Connected. Press Ctrl+C to stop capturing and proceed to saving.\n");

        var cancelRequested = false;
        ConsoleCancelEventHandler? handler = (sender, args) =>
        {
            args.Cancel = true;
            cancelRequested = true;
        };
        Console.CancelKeyPress += handler;

        try
        {
            while (true)
            {
                string? header;
                try
                {
                    header = ReadLine(stream, () => cancelRequested);
                }
                catch (OperationCanceledException)
                {
                    userCancelled = true;
                    break;
                }
                catch (IOException ex)
                {
                    Console.WriteLine($"Connection error while reading header: {ex.Message}");
                    streamClosed = true;
                    break;
                }

                if (header is null)
                {
                    streamClosed = true;
                    break;
                }

                if (cancelRequested)
                {
                    userCancelled = true;
                    break;
                }

                if (header.Equals("PING", StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                if (!header.StartsWith("PCD ", StringComparison.OrdinalIgnoreCase))
                {
                    Console.WriteLine($"Received unexpected message: '{header}'");
                    continue;
                }

                string[] parts = header.Split(' ', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
                if (parts.Length < 2 ||
                    !int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out int payloadLength) ||
                    payloadLength <= 0)
                {
                    Console.WriteLine($"Malformed PCD header: '{header}'");
                    continue;
                }

                var payload = new byte[payloadLength];
                try
                {
                    bool success = ReadExact(stream, payload, 0, payloadLength, () => cancelRequested);
                    if (!success)
                    {
                        streamClosed = true;
                        break;
                    }
                }
                catch (OperationCanceledException)
                {
                    userCancelled = true;
                    break;
                }
                catch (IOException ex)
                {
                    Console.WriteLine($"Connection error while reading payload: {ex.Message}");
                    streamClosed = true;
                    break;
                }

                aggregatedData.Write(payload, 0, payload.Length);
                frames++;

                long pointsInFrame = payloadLength / (BytesPerFloat * FloatsPerPoint);
                totalPoints += pointsInFrame;
                totalPayloadBytes += payloadLength;

                if (payloadLength % (BytesPerFloat * FloatsPerPoint) != 0)
                {
                    Console.WriteLine($"Warning: payload size {payloadLength} is not divisible by {BytesPerFloat * FloatsPerPoint}; point count may be inaccurate.");
                }

                string frameSize = FormatSize(payloadLength);
                string totalSize = FormatSize(totalPayloadBytes);
                string processMemory = FormatSize(GC.GetTotalMemory(false));

                Console.WriteLine(
                    $"[{DateTime.Now:HH:mm:ss}] Frame {frames}: {pointsInFrame:N0} pts ({frameSize}); total {totalPoints:N0} pts ({totalSize}), process memory ~{processMemory}.");
            }
        }
        finally
        {
            Console.CancelKeyPress -= handler;
        }

        if (streamClosed)
        {
            Console.WriteLine("Stream closed by the server.");
        }
        else if (userCancelled)
        {
            Console.WriteLine("Capture stopped by user request.");
        }

        if (totalPoints == 0)
        {
            Console.WriteLine("No point data captured; nothing to save.");
            return;
        }

        Console.WriteLine();
        Console.Write("Enter output path for the merged PCD file (leave empty to cancel): ");
        string? outputPath = Console.ReadLine();
        if (string.IsNullOrWhiteSpace(outputPath))
        {
            Console.WriteLine("Save cancelled by user.");
            return;
        }

        try
        {
            SavePcd(outputPath, aggregatedData, totalPoints);
            Console.WriteLine($"Saved {totalPoints:N0} points to '{outputPath}'.");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Failed to save PCD file: {ex.Message}");
        }
    }

    private static (string Host, int Port) PromptForEndpoint()
    {
        const string defaultHost = "127.0.0.1";
        const int defaultPort = 23511;

        while (true)
        {
            Console.Write($"Enter LiDAR TCP endpoint [host:port] (default {defaultHost}:{defaultPort}): ");
            string? input = Console.ReadLine();
            if (string.IsNullOrWhiteSpace(input))
            {
                return (defaultHost, defaultPort);
            }

            input = input.Trim();
            if (Uri.TryCreate(input, UriKind.Absolute, out Uri? uri) &&
                uri.Scheme is "tcp" or "http" or "https")
            {
                if (uri.Port <= 0)
                {
                    Console.WriteLine("The URI must include a valid port number.");
                    continue;
                }

                return (uri.Host, uri.Port);
            }

            int lastColon = input.LastIndexOf(':');
            if (lastColon > 0 &&
                int.TryParse(input[(lastColon + 1)..], NumberStyles.Integer, CultureInfo.InvariantCulture, out int port) &&
                port is > 0 and <= 65535)
            {
                string host = input[..lastColon].Trim();
                if (host.StartsWith('[') && host.EndsWith(']'))
                {
                    host = host[1..^1];
                }

                if (string.IsNullOrEmpty(host))
                {
                    Console.WriteLine("Host part cannot be empty.");
                    continue;
                }

                return (host, port);
            }

            Console.WriteLine("Input must be in the form host:port or a valid URI (e.g., tcp://127.0.0.1:23511).");
        }
    }

    private static string? ReadLine(NetworkStream stream, Func<bool> cancellationRequested)
    {
        var buffer = new List<byte>();
        while (true)
        {
            if (cancellationRequested())
            {
                throw new OperationCanceledException();
            }

            int value;
            try
            {
                value = stream.ReadByte();
            }
            catch (IOException ex) when (IsTimeout(ex))
            {
                if (cancellationRequested())
                {
                    throw new OperationCanceledException();
                }

                continue;
            }

            if (value == -1)
            {
                return buffer.Count == 0 ? null : Encoding.ASCII.GetString(buffer.ToArray());
            }

            if (value == '\n')
            {
                break;
            }

            if (value != '\r')
            {
                buffer.Add((byte)value);
            }
        }

        return Encoding.ASCII.GetString(buffer.ToArray());
    }

    private static bool ReadExact(NetworkStream stream, byte[] buffer, int offset, int count, Func<bool> cancellationRequested)
    {
        int totalRead = 0;
        while (totalRead < count)
        {
            if (cancellationRequested())
            {
                throw new OperationCanceledException();
            }

            int read;
            try
            {
                read = stream.Read(buffer, offset + totalRead, count - totalRead);
            }
            catch (IOException ex) when (IsTimeout(ex))
            {
                if (cancellationRequested())
                {
                    throw new OperationCanceledException();
                }

                continue;
            }

            if (read == 0)
            {
                return false;
            }

            totalRead += read;
        }

        return true;
    }

    private static bool IsTimeout(IOException ex)
    {
        return ex.InnerException is SocketException socketException &&
               socketException.SocketErrorCode is SocketError.TimedOut or SocketError.WouldBlock;
    }

    private static string FormatSize(long bytes)
    {
        const double scale = 1024.0;
        if (bytes >= scale * scale * scale)
        {
            return string.Format(CultureInfo.InvariantCulture, "{0:F2} GiB", bytes / (scale * scale * scale));
        }

        if (bytes >= scale * scale)
        {
            return string.Format(CultureInfo.InvariantCulture, "{0:F2} MiB", bytes / (scale * scale));
        }

        if (bytes >= scale)
        {
            return string.Format(CultureInfo.InvariantCulture, "{0:F1} KiB", bytes / scale);
        }

        return string.Format(CultureInfo.InvariantCulture, "{0} B", bytes);
    }

    private static void SavePcd(string path, MemoryStream payload, long totalPoints)
    {
        payload.Flush();
        payload.Position = 0;

        var headerBuilder = new StringBuilder();
        headerBuilder.AppendLine("# .PCD v0.7 - Generated by _PCDclient");
        headerBuilder.AppendLine("VERSION 0.7");
        headerBuilder.AppendLine("FIELDS x y z intensity");
        headerBuilder.AppendLine("SIZE 4 4 4 4");
        headerBuilder.AppendLine("TYPE F F F F");
        headerBuilder.AppendLine("COUNT 1 1 1 1");
        headerBuilder.AppendLine($"WIDTH {totalPoints}");
        headerBuilder.AppendLine("HEIGHT 1");
        headerBuilder.AppendLine("VIEWPOINT 0 0 0 1 0 0 0");
        headerBuilder.AppendLine($"POINTS {totalPoints}");
        headerBuilder.AppendLine("DATA binary");

        byte[] headerBytes = Encoding.ASCII.GetBytes(headerBuilder.ToString());

        using var file = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None);
        file.Write(headerBytes, 0, headerBytes.Length);
        payload.CopyTo(file);
    }
}
