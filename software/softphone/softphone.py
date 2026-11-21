#!/usr/bin/env python3
"""
Softphone for the ESP32 rotary-phone exchange.

This PC joins the same UDP network as the ESP32 nodes. The default
extension is 111. Audio is 8 kHz, 160-sample (20 ms) little-endian PCM
on UDP port 5000. See docs/protocol.md.

Audio flows only after the call reaches "connected". On an ESP32 node
the handset must be off-hook; the firmware drops microphone audio
otherwise. The phone's DAC keeps only the top 8 bits of each sample, so
a quiet laptop mic is easy to miss without the transmit gain below.

Examples:
    python3 softphone.py
    python3 softphone.py --number 222
    python3 softphone.py --bind 192.168.1.50
    python3 softphone.py --tx-gain 8 --list-devices

Commands:
    c 456   Call phone 456
    a       Answer an incoming call
    h       Hang up
    t       Toggle a 1 kHz test tone in place of the microphone
    g 8     Set microphone gain (the phone hears gain in the top 8 bits)
    s       Print audio and call status
    p       Send discovery hello
    l       List discovered peers
    q       Quit
"""

import argparse
import socket
import struct
import threading
import time

import numpy as np
import sounddevice as sd


MAGIC = 0x544C5032  # "TLP2"
VERSION = 2
DEFAULT_NUMBER = 111
UDP_PORT = 5000
# Use the subnet broadcast, such as "192.168.1.255", if the router drops
# limited broadcasts to 255.255.255.255. Match DISCOVERY_IP in the sketch.
DISCOVERY_ADDR = "255.255.255.255"

SAMPLE_RATE = 8000
FRAME_SAMPLES = 160
FRAME_SECONDS = FRAME_SAMPLES / SAMPLE_RATE

PKT_HELLO = 1
PKT_AUDIO = 2
PKT_CONTROL = 3

CTRL_CALL = 1
CTRL_RINGING = 2
CTRL_ANSWER = 3
CTRL_HANGUP = 4
CTRL_BUSY = 5
CTRL_REJECT = 6
CTRL_PING = 7

# Matches PacketHeader / HelloPacket / ControlPacket in the sketch.
HEADER = struct.Struct("<IBBHHH")
HELLO_FULL = struct.Struct("<IBBHHH H 4s 2s")
CONTROL_FULL = struct.Struct("<IBBHHH BB HHH")

TONE_HZ = 1000.0
TONE_AMPLITUDE = 14000  # survives the phone DAC's >> 8 (~55/128)
# Peaks below this are only a few DAC steps and are hard to hear.
AUDIBLE_TX_PEAK = 2048


class Peer:
    def __init__(self, number, ip):
        self.number = number
        self.ip = ip
        self.last_seen = time.monotonic()

    def __repr__(self):
        return f"{self.number:03d} @ {self.ip}"


class SampleRing:
    """Fixed float32 ring. Writers drop the oldest samples on overflow."""

    def __init__(self, capacity):
        self.buf = np.zeros(int(capacity), dtype=np.float32)
        self.cap = int(capacity)
        self.read_pos = 0
        self.write_pos = 0
        self.count = 0
        self.lock = threading.Lock()

    def write(self, samples):
        samples = np.asarray(samples, dtype=np.float32).reshape(-1)
        if samples.size == 0:
            return 0
        with self.lock:
            if samples.size >= self.cap:
                samples = samples[-self.cap:]
            overflow = max(0, samples.size - (self.cap - self.count))
            if overflow:
                self.read_pos = (self.read_pos + overflow) % self.cap
                self.count -= overflow
            n = int(samples.size)
            first = min(n, self.cap - self.write_pos)
            self.buf[self.write_pos:self.write_pos + first] = samples[:first]
            rest = n - first
            if rest:
                self.buf[:rest] = samples[first:]
            self.write_pos = (self.write_pos + n) % self.cap
            self.count += n
            return overflow

    def available(self):
        with self.lock:
            return self.count

    def read(self, n):
        out = np.zeros(int(n), dtype=np.float32)
        with self.lock:
            take = min(int(n), self.count)
            if take:
                first = min(take, self.cap - self.read_pos)
                out[:first] = self.buf[self.read_pos:self.read_pos + first]
                rest = take - first
                if rest:
                    out[first:take] = self.buf[:rest]
                self.read_pos = (self.read_pos + take) % self.cap
                self.count -= take
        return out, take < n

    def clear(self):
        with self.lock:
            self.read_pos = 0
            self.write_pos = 0
            self.count = 0


class StreamResampler:
    """Linear resampler that carries leftover input across callbacks."""

    def __init__(self, in_rate, out_rate):
        self.in_rate = float(in_rate)
        self.out_rate = float(out_rate)
        self.held = np.zeros(0, dtype=np.float32)
        self.origin = 0.0
        self.received = 0.0
        self.next_out = 0.0

    def process(self, samples):
        samples = np.asarray(samples, dtype=np.float32).reshape(-1)
        if samples.size == 0:
            return samples
        if self.in_rate == self.out_rate:
            return samples.copy()

        if self.held.size == 0:
            self.origin = self.received
        self.held = (
            samples.copy()
            if self.held.size == 0
            else np.concatenate((self.held, samples))
        )
        self.received += samples.size

        step = self.in_rate / self.out_rate
        last_index = self.origin + self.held.size - 1
        outputs = []
        while self.next_out + 1.0 <= last_index:
            rel = self.next_out - self.origin
            i = int(rel)
            frac = rel - i
            outputs.append(
                self.held[i] * (1.0 - frac) + self.held[i + 1] * frac
            )
            self.next_out += step

        drop = int(np.floor(self.next_out - self.origin))
        if drop > 0:
            if drop >= self.held.size:
                self.held = np.zeros(0, dtype=np.float32)
                self.origin = self.received
            else:
                self.held = self.held[drop:]
                self.origin += drop

        if not outputs:
            return np.zeros(0, dtype=np.float32)
        return np.asarray(outputs, dtype=np.float32)


class SoftPhone:
    def __init__(self, bind_ip, tx_gain, rx_gain, input_device, output_device, number):
        self.bind_ip = bind_ip
        self.tx_gain = float(tx_gain)
        self.rx_gain = float(rx_gain)
        self.input_device = input_device
        self.output_device = output_device
        self.number = int(number)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((bind_ip, UDP_PORT))
        self.sock.settimeout(0.1)

        self.running = True
        self.print_lock = threading.Lock()

        self.peers = {}
        self.peers_lock = threading.Lock()

        self.remote_number = None
        self.remote_ip = None
        self.call_state = "idle"
        self.state_lock = threading.Lock()

        self.seq = 0
        self.test_tone = False
        self.tone_phase = 0

        self.in_rate = SAMPLE_RATE
        self.out_rate = SAMPLE_RATE
        self.tx_resampler = StreamResampler(SAMPLE_RATE, SAMPLE_RATE)
        self.rx_resampler = StreamResampler(SAMPLE_RATE, SAMPLE_RATE)
        self.tx_ring = SampleRing(SAMPLE_RATE)
        self.rx_ring = SampleRing(SAMPLE_RATE * 2)
        self.rx_primed = False

        self.stats_lock = threading.Lock()
        self.tx_packets = 0
        self.rx_packets = 0
        self.tx_underruns = 0
        self.rx_underruns = 0
        self.rx_gaps = 0
        self.tx_errors = 0
        self.in_overflows = 0
        self.out_overflows = 0
        self.last_rx_seq = None
        self.last_rx_time = 0.0
        self.last_tx_time = 0.0
        self.interval_tx_peak = 0
        self.interval_rx_peak = 0
        self.last_tx_peak = 0
        self.last_rx_peak = 0
        self.rate_tx = 0
        self.rate_rx = 0
        self.rate_time = time.monotonic()
        self.announced_tx = False
        self.announced_rx = False

        self.input_name = "unavailable"
        self.output_name = "unavailable"

    def log(self, message):
        with self.print_lock:
            print(message, flush=True)

    def local_ip(self):
        if self.bind_ip not in ("0.0.0.0", ""):
            return self.bind_ip
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
        except OSError:
            return "0.0.0.0"
        finally:
            probe.close()

    def send_hello(self):
        ip_bytes = socket.inet_aton(self.local_ip())
        packet = HELLO_FULL.pack(
            MAGIC, PKT_HELLO, VERSION, 0, 0, 0,
            self.number, ip_bytes, b"\x00\x00",
        )
        try:
            self.sock.sendto(packet, (DISCOVERY_ADDR, UDP_PORT))
        except OSError:
            pass

    def send_control(self, ip, code, caller, called):
        packet = CONTROL_FULL.pack(
            MAGIC, PKT_CONTROL, VERSION, 0, 0, 0,
            code, 0, caller, called, 0,
        )
        try:
            self.sock.sendto(packet, (ip, UDP_PORT))
        except OSError as exc:
            self.log(f"Control send failed: {exc}")

    def send_audio(self, samples):
        with self.state_lock:
            ip = self.remote_ip
            connected = self.call_state == "connected"
        if not connected or not ip:
            return

        samples = np.asarray(samples, dtype=np.int16).reshape(-1)
        if samples.size == 0:
            return

        header = HEADER.pack(
            MAGIC, PKT_AUDIO, VERSION,
            self.seq & 0xFFFF, samples.size & 0xFFFF, 0,
        )
        self.seq = (self.seq + 1) & 0xFFFF
        try:
            self.sock.sendto(header + samples.astype("<i2").tobytes(), (ip, UDP_PORT))
        except OSError:
            with self.stats_lock:
                self.tx_errors += 1
            return

        peak = int(np.max(np.abs(samples))) if samples.size else 0
        now = time.monotonic()
        with self.stats_lock:
            self.tx_packets += 1
            self.last_tx_time = now
            self.last_tx_peak = peak
            if peak > self.interval_tx_peak:
                self.interval_tx_peak = peak
            first = not self.announced_tx
            self.announced_tx = True
        if first:
            if peak <= 0:
                self.log("TX packets started, level is still silence")
            else:
                self.log(
                    f"TX packets started  peak {peak}  "
                    f"phone DAC {min(127, peak >> 8)}/128"
                )

    def discover_peer(self, number, ip):
        if number == self.number:
            return
        with self.peers_lock:
            old = self.peers.get(number)
            if old is None or old.ip != ip:
                self.peers[number] = Peer(number, ip)
                self.log(f"PEER DISCOVERED: {number:03d} @ {ip}")
            else:
                old.last_seen = time.monotonic()

    def handle_hello(self, data, addr):
        if len(data) < HELLO_FULL.size:
            return
        (
            magic, _ptype, version, _seq, _samples, _reserved,
            number, ip_bytes, _reserved2,
        ) = HELLO_FULL.unpack_from(data)
        if magic != MAGIC or version != VERSION:
            return
        advertised = socket.inet_ntoa(ip_bytes)
        if advertised == "0.0.0.0":
            advertised = addr[0]
        self.discover_peer(number, advertised)

    def handle_control(self, data, addr):
        if len(data) < CONTROL_FULL.size:
            return
        (
            magic, _ptype, version, _seq, _samples, _reserved,
            code, _value, caller, called, _reserved2,
        ) = CONTROL_FULL.unpack_from(data)
        if magic != MAGIC or version != VERSION:
            return

        if code == CTRL_CALL:
            if called != self.number:
                return
            with self.state_lock:
                if self.call_state != "idle":
                    self.send_control(addr[0], CTRL_BUSY, self.number, caller)
                    self.log(f"Rejected call from {caller:03d} (busy)")
                    return
                self.remote_number = caller
                self.remote_ip = addr[0]
                self.call_state = "incoming"
            self.log(
                f"INCOMING CALL FROM {caller:03d}. "
                "Press 'a' to answer or 'h' to hang up."
            )
            self.send_control(addr[0], CTRL_RINGING, self.number, caller)

        elif code == CTRL_RINGING:
            with self.state_lock:
                if self.call_state == "outgoing":
                    self.log(f"{self.remote_number:03d} is ringing...")

        elif code == CTRL_ANSWER:
            connected = False
            with self.state_lock:
                if self.call_state == "outgoing":
                    self.call_state = "connected"
                    connected = True
            if connected:
                self.log(
                    "CALL CONNECTED. Audio is live. "
                    "Press 't' for a 1 kHz test tone, 's' for levels."
                )

        elif code == CTRL_HANGUP:
            self.end_call("Remote hung up")

        elif code == CTRL_BUSY:
            number = self.remote_number
            self.end_call(
                f"{number:03d} is busy" if number is not None else "Remote is busy"
            )

        elif code == CTRL_REJECT:
            self.end_call("Call rejected")

    def handle_audio(self, data, addr, seq, nsamples):
        payload = data[HEADER.size:]
        available = len(payload) // 2
        count = min(int(nsamples), available)
        if count <= 0:
            return

        audio = np.frombuffer(payload[:count * 2], dtype="<i2").astype(np.int16)
        peak = int(np.max(np.abs(audio))) if audio.size else 0
        now = time.monotonic()
        gap = 0
        with self.stats_lock:
            if self.last_rx_seq is not None:
                expected = (self.last_rx_seq + 1) & 0xFFFF
                if seq != expected:
                    gap = (seq - expected) & 0xFFFF
                    self.rx_gaps += gap
            self.last_rx_seq = seq
            self.rx_packets += 1
            self.last_rx_time = now
            self.last_rx_peak = peak
            if peak > self.interval_rx_peak:
                self.interval_rx_peak = peak
            first = not self.announced_rx
            self.announced_rx = True

        if first:
            self.log(f"RX audio started from {addr[0]}  {count} samples  peak {peak}")

        with self.state_lock:
            remote = self.remote_ip
            connected = self.call_state == "connected"
        if connected and remote and addr[0] != remote:
            return

        played = (audio.astype(np.float32) / 32768.0) * self.rx_gain
        self.rx_ring.write(self.rx_resampler.process(played))

    def end_call(self, reason):
        with self.state_lock:
            was_idle = self.call_state == "idle"
            self.call_state = "idle"
            self.remote_number = None
            self.remote_ip = None
        if was_idle:
            return
        with self.stats_lock:
            self.announced_tx = False
            self.announced_rx = False
            self.last_rx_seq = None
        self.tx_ring.clear()
        self.rx_ring.clear()
        self.rx_primed = False
        self.log(reason)

    def network_thread(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) < HEADER.size:
                continue
            magic, ptype, version, seq, nsamples, _reserved = HEADER.unpack_from(data)
            if magic != MAGIC or version != VERSION:
                continue

            if ptype == PKT_HELLO:
                self.handle_hello(data, addr)
            elif ptype == PKT_CONTROL:
                self.handle_control(data, addr)
            elif ptype == PKT_AUDIO:
                self.handle_audio(data, addr, seq, nsamples)

    def list_peers(self):
        now = time.monotonic()
        with self.peers_lock:
            peers = sorted(self.peers.values(), key=lambda peer: peer.number)
        if not peers:
            self.log("No peers discovered.")
            return
        self.log("Discovered peers:")
        for peer in peers:
            age = now - peer.last_seen
            self.log(f"  {peer.number:03d}  {peer.ip}  heard {age:.0f}s ago")

    def call(self, number):
        with self.peers_lock:
            peer = self.peers.get(number)
        if peer is None:
            self.log(
                f"Phone {number:03d} has not been discovered yet. "
                "Wait for HELLO or press 'p'."
            )
            return
        with self.state_lock:
            if self.call_state != "idle":
                self.log("Already in a call.")
                return
            self.remote_number = number
            self.remote_ip = peer.ip
            self.call_state = "outgoing"
        self.log(f"CALLING {number:03d} @ {peer.ip}")
        self.log("No audio until the handset is answered.")
        self.send_control(peer.ip, CTRL_CALL, self.number, number)

    def answer(self):
        with self.state_lock:
            if self.call_state != "incoming":
                self.log("No incoming call.")
                return
            remote_ip = self.remote_ip
            remote_number = self.remote_number
            self.call_state = "connected"
        self.log(f"ANSWERING {remote_number:03d}")
        self.log("Audio is live. Press 't' for a 1 kHz test tone, 's' for levels.")
        self.send_control(remote_ip, CTRL_ANSWER, self.number, remote_number)

    def hangup(self):
        with self.state_lock:
            if self.call_state == "idle":
                self.log("No call.")
                return
            remote_ip = self.remote_ip
            remote_number = self.remote_number
            self.call_state = "idle"
            self.remote_ip = None
            self.remote_number = None
        if remote_ip:
            self.send_control(remote_ip, CTRL_HANGUP, self.number, remote_number or 0)
        with self.stats_lock:
            self.announced_tx = False
            self.announced_rx = False
            self.last_rx_seq = None
        self.tx_ring.clear()
        self.rx_ring.clear()
        self.rx_primed = False
        self.log("HANG UP")

    def make_tone(self):
        n = FRAME_SAMPLES
        idx = np.arange(n, dtype=np.float64) + self.tone_phase
        wave = np.sin(2.0 * np.pi * TONE_HZ * idx / SAMPLE_RATE)
        self.tone_phase = (self.tone_phase + n) % SAMPLE_RATE
        return np.clip(wave * TONE_AMPLITUDE, -32767, 32767).astype(np.int16)

    def take_mic_frame(self):
        block, short = self.tx_ring.read(FRAME_SAMPLES)
        if short:
            with self.stats_lock:
                self.tx_underruns += 1
        gained = np.clip(block * self.tx_gain * 32767.0, -32767, 32767)
        return gained.astype(np.int16)

    def send_tx_frame(self):
        with self.state_lock:
            connected = self.call_state == "connected"
        if not connected:
            self.tx_ring.clear()
            return
        if self.test_tone:
            self.tx_ring.clear()
            self.send_audio(self.make_tone())
            return
        self.send_audio(self.take_mic_frame())

    def tx_thread(self):
        next_t = time.monotonic()
        while self.running:
            next_t += FRAME_SECONDS
            self.send_tx_frame()
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:
                next_t = time.monotonic()

    def status_line(self, tx_pps, rx_pps, tx_peak, rx_peak):
        with self.state_lock:
            state = self.call_state
            number = self.remote_number
        who = f"{number:03d}" if number is not None else "----"
        dac = min(127, int(tx_peak) >> 8)
        source = "tone" if self.test_tone else "mic"
        if state != "connected":
            return (
                f"[{state} {who}] no audio yet — media starts when the call connects"
            )
        if tx_pps >= 30 and rx_pps >= 30:
            path = "both directions"
        elif tx_pps >= 30:
            path = "TX only — no packets from the phone (handset must stay off-hook)"
        elif rx_pps >= 30:
            path = "RX only, nothing sent"
        else:
            path = "no packets"
        tx_hear = "audible" if tx_peak >= AUDIBLE_TX_PEAK else "too quiet for the phone DAC"
        rx_hear = "signal" if rx_peak >= 500 else "quiet"
        return (
            f"[connected {who}] {path}  "
            f"tx {tx_pps:.0f}/s {source} peak {tx_peak} DAC {dac}/128 ({tx_hear})  "
            f"rx {rx_pps:.0f}/s peak {rx_peak} ({rx_hear})"
        )

    def print_status(self, periodic=False):
        now = time.monotonic()
        with self.stats_lock:
            tx = self.tx_packets
            rx = self.rx_packets
            tx_peak = self.interval_tx_peak if periodic else self.last_tx_peak
            rx_peak = self.interval_rx_peak if periodic else self.last_rx_peak
            if periodic:
                self.interval_tx_peak = 0
                self.interval_rx_peak = 0
            gaps = self.rx_gaps
            tx_underruns = self.tx_underruns
            rx_underruns = self.rx_underruns
            tx_errors = self.tx_errors
            last_rx = self.last_rx_time
            last_tx = self.last_tx_time
            prev_tx = self.rate_tx
            prev_rx = self.rate_rx
            prev_t = self.rate_time
            if periodic:
                self.rate_tx = tx
                self.rate_rx = rx
                self.rate_time = now

        dt = max(now - prev_t, 1e-3)
        tx_pps = (tx - prev_tx) / dt if periodic else 0.0
        rx_pps = (rx - prev_rx) / dt if periodic else 0.0
        if not periodic:
            # Instant rates are meaningless; show whether packets are recent.
            tx_pps = 50.0 if last_tx and now - last_tx < 0.5 else 0.0
            rx_pps = 50.0 if last_rx and now - last_rx < 0.5 else 0.0

        self.log(self.status_line(tx_pps, rx_pps, tx_peak, rx_peak))
        if not periodic:
            rx_age = f"{now - last_rx:.1f}s" if last_rx else "never"
            tx_age = f"{now - last_tx:.1f}s" if last_tx else "never"
            self.log(
                f"  totals tx {tx} ({tx_age} ago)  rx {rx} ({rx_age} ago)  "
                f"rx gaps {gaps}  tx underrun {tx_underruns}  "
                f"rx underrun {rx_underruns}  tx errors {tx_errors}"
            )
            self.log(
                f"  gain {self.tx_gain:g}  rx gain {self.rx_gain:g}  "
                f"mic {self.in_rate:.0f} Hz ({self.input_name})  "
                f"speaker {self.out_rate:.0f} Hz ({self.output_name})"
            )

    def status_thread(self):
        while self.running:
            for _ in range(20):
                if not self.running:
                    return
                time.sleep(0.1)
            with self.state_lock:
                state = self.call_state
            if state == "idle":
                with self.stats_lock:
                    self.rate_tx = self.tx_packets
                    self.rate_rx = self.rx_packets
                    self.rate_time = time.monotonic()
                    self.interval_tx_peak = 0
                    self.interval_rx_peak = 0
                self.rx_primed = False
                continue
            self.print_status(periodic=True)

    def audio_input_callback(self, indata, frames, time_info, status):
        if status:
            with self.stats_lock:
                self.in_overflows += 1
        with self.state_lock:
            connected = self.call_state == "connected"
        if not connected or self.test_tone:
            return
        try:
            samples = self.tx_resampler.process(indata[:, 0])
            if samples.size:
                self.tx_ring.write(samples)
        except Exception:
            with self.stats_lock:
                self.in_overflows += 1

    def audio_output_callback(self, outdata, frames, time_info, status):
        if status:
            with self.stats_lock:
                self.out_overflows += 1
        with self.state_lock:
            connected = self.call_state == "connected"
        if not connected:
            self.rx_primed = False
            outdata.fill(0)
            return
        try:
            # Hold a short cushion so the start of a call is not a string of gaps.
            if not self.rx_primed:
                if self.rx_ring.available() < int(self.out_rate * 0.08):
                    outdata.fill(0)
                    return
                self.rx_primed = True
            samples, short = self.rx_ring.read(frames)
            if short:
                with self.stats_lock:
                    self.rx_underruns += 1
            outdata[:, 0] = np.clip(samples, -1.0, 1.0)
        except Exception:
            outdata.fill(0)

    def open_stream(self, kind, device, callback):
        """Open at 8 kHz when the device allows it, otherwise at its native rate."""
        info = sd.query_devices(device, kind)
        native = int(info["default_samplerate"])
        name = info["name"]
        last_error = None
        rates = [SAMPLE_RATE] if native == SAMPLE_RATE else [SAMPLE_RATE, native]
        for rate in rates:
            stream = None
            try:
                kwargs = dict(
                    samplerate=rate,
                    channels=1,
                    dtype="float32",
                    blocksize=0,
                    device=device,
                    callback=callback,
                )
                stream = (
                    sd.InputStream(**kwargs)
                    if kind == "input"
                    else sd.OutputStream(**kwargs)
                )
                # Install the matching resampler before callbacks can run.
                if kind == "input":
                    self.in_rate = float(rate)
                    self.tx_resampler = StreamResampler(rate, SAMPLE_RATE)
                else:
                    self.out_rate = float(rate)
                    self.rx_resampler = StreamResampler(SAMPLE_RATE, rate)
                stream.start()
                return stream, float(rate), name
            except Exception as exc:
                last_error = exc
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass
        self.log(f"{kind.capitalize()} unavailable ({name}): {last_error}")
        return None, float(native), name

    def keyboard_thread(self):
        self.log("")
        self.log(f"SOFTPHONE {self.number:03d}")
        self.log("----------------")
        self.log("  c 456 = call phone 456")
        self.log("  a     = answer incoming call")
        self.log("  h     = hang up")
        self.log("  t     = toggle 1 kHz test tone")
        self.log("  g 8   = microphone gain")
        self.log("  s     = audio status")
        self.log("  l     = list discovered peers")
        self.log("  p     = send discovery HELLO")
        self.log("  q     = quit")
        self.log("")

        while self.running:
            try:
                command = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                command = "q"

            parts = command.split()
            if not parts:
                continue
            cmd = parts[0].lower()

            if cmd == "c":
                if len(parts) != 2 or not parts[1].isdigit():
                    self.log("Usage: c 456")
                    continue
                number = int(parts[1])
                if not 0 <= number <= 999:
                    self.log("Phone number must be 000-999.")
                    continue
                self.call(number)
            elif cmd == "a":
                self.answer()
            elif cmd == "h":
                self.hangup()
            elif cmd == "t":
                self.test_tone = not self.test_tone
                self.tone_phase = 0
                if self.test_tone:
                    self.log(
                        f"Test tone ON ({TONE_HZ:.0f} Hz). "
                        "The phone should hear a steady beep while connected."
                    )
                else:
                    self.log("Test tone OFF. Sending the microphone.")
            elif cmd == "g":
                if len(parts) != 2:
                    self.log(f"Microphone gain is {self.tx_gain:g}. Usage: g 8")
                    continue
                try:
                    gain = float(parts[1])
                except ValueError:
                    self.log("Usage: g 8")
                    continue
                if gain <= 0:
                    self.log("Gain must be greater than 0.")
                    continue
                self.tx_gain = gain
                self.log(f"Microphone gain set to {gain:g}")
            elif cmd == "s":
                self.print_status(periodic=False)
            elif cmd == "l":
                self.list_peers()
            elif cmd == "p":
                self.log("Sending HELLO...")
                self.send_hello()
            elif cmd == "q":
                self.running = False
                with self.state_lock:
                    in_call = self.call_state != "idle"
                if in_call:
                    self.hangup()
                break
            else:
                self.log("Unknown command.")

    def run(self):
        self.log("ESP32 rotary exchange softphone")
        self.log(f"Phone number: {self.number:03d}")
        self.log(f"Listening on {self.bind_ip}:{UDP_PORT}")
        self.log(f"Advertising {self.local_ip()} as {self.number:03d}")
        self.log(
            f"TX gain {self.tx_gain:g} (phone DAC uses the top 8 bits; "
            f"peak {AUDIBLE_TX_PEAK}+ is the useful range)"
        )

        in_stream, self.in_rate, self.input_name = self.open_stream(
            "input", self.input_device, self.audio_input_callback
        )
        out_stream, self.out_rate, self.output_name = self.open_stream(
            "output", self.output_device, self.audio_output_callback
        )
        self.tx_ring.clear()
        if in_stream is not None:
            self.log(f"Microphone: {self.input_name} at {self.in_rate:.0f} Hz")
        if out_stream is not None:
            self.log(f"Speaker: {self.output_name} at {self.out_rate:.0f} Hz")

        threads = [
            threading.Thread(target=self.network_thread, daemon=True),
            threading.Thread(target=self.tx_thread, daemon=True),
            threading.Thread(target=self.status_thread, daemon=True),
            threading.Thread(target=self.periodic_hello, daemon=True),
        ]
        for thread in threads:
            thread.start()

        self.send_hello()
        try:
            self.keyboard_thread()
        finally:
            self.running = False
            for stream in (in_stream, out_stream):
                if stream is None:
                    continue
                try:
                    stream.stop()
                    stream.close()
                except Exception:
                    pass
            try:
                self.sock.close()
            except OSError:
                pass

    def periodic_hello(self):
        while self.running:
            for _ in range(50):
                if not self.running:
                    return
                time.sleep(0.1)
            if self.running:
                self.send_hello()


def list_devices():
    print(sd.query_devices())


def main():
    parser = argparse.ArgumentParser(
        description="Softphone for the ESP32 rotary-phone exchange."
    )
    parser.add_argument(
        "--number",
        type=int,
        default=DEFAULT_NUMBER,
        help=f"This computer's 3-digit extension (default: {DEFAULT_NUMBER})",
    )
    parser.add_argument(
        "--bind",
        default="0.0.0.0",
        help="Local bind address (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--tx-gain",
        type=float,
        default=8.0,
        help="Microphone gain before the 8 kHz frames (default: 8)",
    )
    parser.add_argument(
        "--rx-gain",
        type=float,
        default=1.0,
        help="Speaker gain for audio received from the phone (default: 1)",
    )
    parser.add_argument("--input", type=int, default=None, help="Input device id")
    parser.add_argument("--output", type=int, default=None, help="Output device id")
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="Print PortAudio devices and exit",
    )
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return
    if not 0 <= args.number <= 999:
        parser.error("Phone number must be 000-999.")

    phone = SoftPhone(
        args.bind,
        args.tx_gain,
        args.rx_gain,
        args.input,
        args.output,
        args.number,
    )
    try:
        phone.run()
    except KeyboardInterrupt:
        phone.running = False


if __name__ == "__main__":
    main()
