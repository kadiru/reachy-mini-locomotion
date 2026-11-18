// Attach to an active GameObject in the existing Quest Unity scene.
// New LAN transport; preserves Emcee's angle convention, not its unknown robot mapping.
// No Agora dependency. Do not commit a scene containing the private shared token.
using System;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class ReachyRecordingSender : MonoBehaviour
{
    public string recorderHost = "REPLACE_WITH_MAC_LAN_IP";
    public int recorderPort = 8765;
    [Tooltip("64 lowercase hex characters from the local token file. Keep private.")]
    public string sharedKeyHex = "";
    [Range(10, 50)] public int sendRateHz = 50;
    private UdpClient udp;
    private byte[] key;
    private string senderId;
    private string challenge = "";
    private long number = 0;
    private double nextSend;

    [Serializable] private class Packet
    {
        public long number;
        public bool teleop_engaged, a_temi_engaged, b_reachy_engaged, funny_engaged;
        public float pitch_deg, roll_deg, yaw_deg, stick_x, stick_y;
    }
    [Serializable] private class Message
    {
        public string protocol = "quest-recording-v1";
        public string sender_id;
        public string challenge;
        public double source_time_s;
        public bool tracking_valid;
        public bool synthetic = false;
        public Packet packet;
    }
    [Serializable] private class Challenge
    {
        public string protocol;
        public string challenge;
    }

    private void OnEnable()
    {
        if (sharedKeyHex.Length != 64 || Array.Exists(sharedKeyHex.ToCharArray(), c => !"0123456789abcdef".Contains(c.ToString())))
        {
            Debug.LogError("Reachy recorder: private 64-character lowercase token required.");
            enabled = false;
            return;
        }
        key = Encoding.ASCII.GetBytes(sharedKeyHex);
        senderId = Guid.NewGuid().ToString("N");
        udp = new UdpClient();
        udp.Connect(recorderHost, recorderPort);
        nextSend = Time.realtimeSinceStartupAsDouble;
    }

    private string Sign(byte[] body)
    {
        using (var mac = new HMACSHA256(key))
            return BitConverter.ToString(mac.ComputeHash(body)).Replace("-", "").ToLowerInvariant();
    }

    private bool EqualSignature(string a, string b)
    {
        if (a.Length != b.Length) return false;
        int diff = 0;
        for (int i = 0; i < a.Length; i++) diff |= a[i] ^ b[i];
        return diff == 0;
    }

    private void ReceiveChallenges()
    {
        // Connected UDP only receives responses from the configured recorder.
        for (int i = 0; udp != null && udp.Available > 0 && i < 32; i++)
        {
            System.Net.IPEndPoint endpoint = null;
            byte[] data = udp.Receive(ref endpoint);
            if (data.Length < 66 || data.Length > 8192 || data[64] != (byte)'\n') continue;
            byte[] body = new byte[data.Length - 65];
            Array.Copy(data, 65, body, 0, body.Length);
            if (!EqualSignature(Encoding.ASCII.GetString(data, 0, 64), Sign(body))) continue;
            Challenge response = JsonUtility.FromJson<Challenge>(Encoding.UTF8.GetString(body));
            if (response != null && response.protocol == "quest-challenge-v1") challenge = response.challenge;
        }
    }

    private static float Wrap(float degrees) { return Mathf.Repeat(degrees + 180f, 360f) - 180f; }

    private void Send(bool forceInvalid)
    {
        if (udp == null) return;
        InputDevice hand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        bool tracked = false, trigger = false, a = false, b = false, grip = false;
        Quaternion q = Quaternion.identity;
        Vector2 stick = Vector2.zero;
        bool valid = !forceInvalid && hand.isValid;
        valid &= hand.TryGetFeatureValue(CommonUsages.isTracked, out tracked) && tracked;
        valid &= hand.TryGetFeatureValue(CommonUsages.deviceRotation, out q);
        valid &= hand.TryGetFeatureValue(CommonUsages.triggerButton, out trigger);
        valid &= hand.TryGetFeatureValue(CommonUsages.primaryButton, out a);
        valid &= hand.TryGetFeatureValue(CommonUsages.secondaryButton, out b);
        valid &= hand.TryGetFeatureValue(CommonUsages.gripButton, out grip);
        hand.TryGetFeatureValue(CommonUsages.primary2DAxis, out stick);
        Vector3 e = q.eulerAngles;
        Packet packet = new Packet {
            number = number++, teleop_engaged = valid && trigger,
            a_temi_engaged = valid && a, b_reachy_engaged = valid && b,
            funny_engaged = valid && grip,
            pitch_deg = valid ? -(Wrap(e.x) + 45f) : 0f,
            roll_deg = valid ? Wrap(e.z) : 0f, yaw_deg = valid ? Wrap(e.y) : 0f,
            stick_x = valid ? Mathf.Clamp(stick.x, -1f, 1f) : 0f,
            stick_y = valid ? Mathf.Clamp(stick.y, -1f, 1f) : 0f
        };
        Message message = new Message {
            sender_id = senderId, challenge = challenge,
            source_time_s = Time.realtimeSinceStartupAsDouble,
            tracking_valid = valid, packet = packet
        };
        byte[] body = Encoding.UTF8.GetBytes(JsonUtility.ToJson(message));
        byte[] wire = Encoding.UTF8.GetBytes(Sign(body) + "\n" + Encoding.UTF8.GetString(body));
        udp.Send(wire, wire.Length);
    }

    private void Update()
    {
        if (udp == null) return;
        try
        {
            ReceiveChallenges();
            double now = Time.realtimeSinceStartupAsDouble;
            if (now >= nextSend)
            {
                nextSend = now + 1.0 / sendRateHz; // no catch-up bursts of old poses
                Send(false);
            }
        }
        catch (Exception ex)
        {
            // Network loss cannot retain a remote lease; host/guardian expire it.
            Debug.LogWarning("Reachy recorder transport: " + ex.GetType().Name);
        }
    }
    private void OnApplicationPause(bool paused)
    {
        if (paused) { try { Send(true); } catch { } }
    }
    private void OnDisable()
    {
        try { Send(true); } catch { }
        if (udp != null) { udp.Close(); udp = null; }
    }
}
