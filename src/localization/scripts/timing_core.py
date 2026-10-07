#!/usr/bin/python3
"""Read-only host clock checks and bounded timestamp statistics, without ROS imports."""
import calendar
from collections import deque
import glob
import math
import os
import re
import shutil
import subprocess
import time


def load_config(path):
    import yaml
    with open(path, encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('configuration must be a mapping')
    return value


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name+' must be finite and positive')
    return value


def count(value, name, minimum=1):
    if isinstance(value, bool):
        raise ValueError(name+' must be an integer')
    numeric = float(value)
    if not math.isfinite(numeric) or int(numeric) != numeric or numeric < minimum:
        raise ValueError('%s must be an integer >= %d' % (name, minimum))
    return int(numeric)


def duration(text):
    """Parse systemd durations such as +5.308ms, 34min 8s, or 228us."""
    text = text.strip().replace('μ', 'u').replace('µ', 'u')
    factors = {'ns': 1e-9, 'us': 1e-6, 'ms': 1e-3, 's': 1., 'sec': 1.,
               'seconds': 1., 'min': 60., 'h': 3600.}
    found = list(re.finditer(r'([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*(ns|us|ms|seconds|sec|min|s|h)', text))
    if not found or re.sub(r'\s+', '', ''.join(text[m.end():n.start()] for m, n in zip(found, found[1:]))):
        raise ValueError('unrecognized duration: '+text)
    if text[:found[0].start()].strip() or text[found[-1].end():].strip():
        raise ValueError('unrecognized duration: '+text)
    return sum(float(m.group(1))*factors[m.group(2)] for m in found)


def key_values(text, separator='='):
    return {key.strip(): value.strip() for key, value in
            (line.split(separator, 1) for line in text.splitlines() if separator in line)}


def parse_timesyncd(properties, status):
    props = key_values(properties)
    values = key_values(status, ':')
    message = props.get('NTPMessage', '')
    match = re.search(r'DestinationTimestamp=([^,}]+)', message)
    if not match:
        raise ValueError('NTP DestinationTimestamp unavailable')
    timestamp = calendar.timegm(time.strptime(match.group(1).strip(), '%a %Y-%m-%d %H:%M:%S UTC'))
    root = values.get('Root distance', '').split(' (')[0]
    return {'backend': 'timesyncd', 'offset_sec': duration(values['Offset']),
            'offset_semantics': 'last NTP packet estimate; not hardware sensor synchronization',
            'root_distance_sec': duration(root), 'sample_epoch_sec': timestamp,
            'poll_interval_sec': duration(props['PollIntervalUSec']),
            'stratum': int(values['Stratum']), 'leap': values['Leap'],
            'source': props.get('ServerName', ''), 'packet_count': int(values['Packet count']),
            'sample_ignored': bool(re.search(r'Ignored=yes\b', message))}


def parse_chrony(text):
    values = key_values(text, ':')
    system = values['System time']
    match = re.fullmatch(r'([\d.eE+-]+) seconds (slow|fast) of NTP time', system)
    if not match:
        raise ValueError('chrony System time unavailable')
    offset = float(match.group(1))*(-1 if match.group(2) == 'slow' else 1)
    delay = float(values['Root delay'].split()[0])
    dispersion = float(values['Root dispersion'].split()[0])
    timestamp = calendar.timegm(time.strptime(values['Ref time (UTC)'], '%a %b %d %H:%M:%S %Y'))
    reference = values['Reference ID']
    return {'backend': 'chrony', 'offset_sec': offset,
            'offset_semantics': 'system clock minus estimated NTP time',
            'root_distance_sec': abs(delay)/2.+dispersion, 'sample_epoch_sec': timestamp,
            'poll_interval_sec': float(values['Update interval'].split()[0]),
            'stratum': int(values['Stratum']), 'leap': values['Leap status'],
            'source': reference, 'local_reference_only': reference.split()[0] in ('7F7F0101', '00000000')}


def evaluate_clock(sample, flags, policy, now):
    if not math.isfinite(now):
        raise ValueError('nonfinite clock check time')
    result = dict(sample)
    failures = []
    if flags.get('NTPSynchronized') != 'yes':
        failures.append('kernel_clock_not_synchronized')
    if flags.get('NTP') != 'yes':
        failures.append('NTP_not_enabled')
    for key in ('offset_sec', 'root_distance_sec', 'sample_epoch_sec', 'poll_interval_sec'):
        if not math.isfinite(float(sample[key])):
            raise ValueError('nonfinite_'+key)
    if not 1 <= sample['stratum'] <= 15 or sample['leap'].lower() not in ('normal', 'insert second', 'delete second'):
        failures.append('invalid_reference_state')
    if sample.get('local_reference_only') or sample.get('sample_ignored'):
        failures.append('untrusted_reference_sample')
    age = now-sample['sample_epoch_sec']
    # Accommodate the daemon's poll interval, but never exceed the configured absolute cap.
    poll = min(sample['poll_interval_sec'], policy['max_poll_interval_sec'])
    limit = min(policy['max_sample_age_sec'], max(policy['min_sample_age_limit_sec'],
                                                poll*policy['max_poll_age_factor']))
    if sample['poll_interval_sec'] <= 0 or sample['poll_interval_sec'] > policy['max_poll_interval_sec']:
        failures.append('poll_interval_out_of_bounds')
    if age > limit:
        failures.append('clock_sample_stale')
    if age < -policy['future_sample_tolerance_sec']:
        failures.append('clock_sample_in_future')
    if abs(sample['offset_sec']) > policy['max_abs_offset_sec']:
        failures.append('clock_offset_exceeds_limit')
    if not 0 <= sample['root_distance_sec'] <= policy['max_root_distance_sec']:
        failures.append('clock_root_distance_exceeds_limit')
    result.update(ready=not failures, status='HOST_CLOCK_READY' if not failures else 'HOST_CLOCK_REJECTED',
                  reasons=failures, sample_age_sec=age, sample_age_limit_sec=limit,
                  flags=flags, hardware_sensor_sync_verified=False)
    return result


def run_readonly(command, timeout):
    env = dict(os.environ, LC_ALL='C', LANG='C', TZ='UTC')
    process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, timeout=timeout, env=env, check=False)
    if process.returncode:
        raise RuntimeError('%s: %s' % (' '.join(command), process.stderr.strip() or 'exit '+str(process.returncode)))
    return process.stdout


def check_host_clock(policy, use_sim_time=False, runner=run_readonly, now=None, which=shutil.which):
    policy = dict(policy)
    for name, value in policy.items():
        if name != 'backend':
            policy[name] = positive(value, 'clock_preflight/'+name)
    if use_sim_time:
        return {'ready': True, 'status': 'REPLAY_UNVERIFIED', 'reasons': [],
                'hardware_sensor_sync_verified': False, 'host_clock_verified': False,
                'exemption': 'explicit use_sim_time: physical synchronization was not checked'}
    result = {'ready': False, 'status': 'HOST_CLOCK_UNAVAILABLE', 'reasons': [],
              'hardware_sensor_sync_verified': False, 'host_clock_verified': False,
              'pps_devices': sorted(glob.glob('/dev/pps*')), 'pps_discipline_verified': False}
    timeout = policy['command_timeout_sec']
    try:
        flags = key_values(runner(['timedatectl', 'show', '--property=NTP', '--property=NTPSynchronized'], timeout))
        backend = policy['backend']
        if backend not in ('auto', 'chrony', 'timesyncd'):
            raise ValueError('unknown clock backend '+backend)
        sample = None
        chrony_error = None
        if backend in ('auto', 'chrony') and which('chronyc'):
            try:
                sample = parse_chrony(runner(['chronyc', '-n', 'tracking'], timeout))
            except (KeyError, ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
                chrony_error = str(error)
                if backend == 'chrony':
                    raise
        if sample is None:
            if backend == 'chrony':
                raise RuntimeError('chronyc is unavailable')
            props = runner(['timedatectl', 'show-timesync', '--all'], timeout)
            status = runner(['timedatectl', 'timesync-status'], timeout)
            sample = parse_timesyncd(props, status)
        result.update(evaluate_clock(sample, flags, policy, time.time() if now is None else now))
        result['host_clock_verified'] = result['ready']
        if chrony_error:
            result['chrony_probe_error'] = chrony_error
    except (KeyError, ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        result['reasons'] = [type(error).__name__+': '+str(error)]
    return result


def distribution(values):
    values = sorted(value for value in values if value is not None and math.isfinite(value))
    if not values:
        return {'count': 0}
    def percentile(q):
        position = (len(values)-1)*q
        lower = int(position)
        upper = min(lower+1, len(values)-1)
        return values[lower]+(values[upper]-values[lower])*(position-lower)
    return {'count': len(values), 'p50_sec': percentile(.5), 'p95_sec': percentile(.95),
            'p99_sec': percentile(.99), 'max_sec': values[-1], 'min_sec': values[0]}


class TimingWindow:
    """Timestamp comparisons only: receipt stamps cannot expose pre-stamp device latency."""
    def __init__(self, window_sec, max_samples, gap_sec, jump_sec):
        self.window_sec = positive(window_sec, 'window_sec')
        self.gap_sec = positive(gap_sec, 'gap_sec')
        self.jump_sec = positive(jump_sec, 'jump_sec')
        self.samples = deque(maxlen=count(max_samples, 'max_samples', 2))
        self.previous = None
        self.invalid_total = 0
        self.received_total = 0
        self.last_received_mono = None

    def add(self, header, receipt_ros, receipt_mono):
        self.received_total += 1
        valid = all(math.isfinite(v) and v > 0 for v in (header, receipt_ros, receipt_mono))
        if not valid:
            self.invalid_total += 1
            if math.isfinite(receipt_mono) and receipt_mono > 0:
                self.last_received_mono = receipt_mono
                self.samples.append({'mono': receipt_mono, 'invalid': 1})
                self.prune(receipt_mono)
            return False
        self.last_received_mono = receipt_mono
        item = {'mono': receipt_mono, 'age': receipt_ros-header, 'invalid': 0}
        if self.previous:
            h, ros, mono = self.previous
            interval, receipt_interval = header-h, receipt_mono-mono
            item.update(interval=interval, receipt_interval=receipt_interval,
                        duplicate=int(interval == 0), backwards=int(interval < 0),
                        gap=int(interval > self.gap_sec or receipt_interval > self.gap_sec),
                        clock_jump=int(abs((receipt_ros-ros)-receipt_interval) > self.jump_sec))
        self.samples.append(item)
        self.previous = (header, receipt_ros, receipt_mono)
        self.prune(receipt_mono)
        return True

    def prune(self, now_mono):
        while self.samples and self.samples[0]['mono'] < now_mono-self.window_sec:
            self.samples.popleft()

    def snapshot(self, now_mono):
        if not math.isfinite(now_mono):
            raise ValueError('nonfinite monotonic time')
        self.prune(now_mono)
        output = {'window_count': len(self.samples), 'received_total': self.received_total,
                  'invalid_total': self.invalid_total,
                  'last_receipt_age_sec': None if self.last_received_mono is None else now_mono-self.last_received_mono}
        for key in ('invalid', 'duplicate', 'backwards', 'gap', 'clock_jump'):
            output[key+'_count'] = sum(row.get(key, 0) for row in self.samples)
        for key in ('age', 'interval', 'receipt_interval'):
            output[key] = distribution(row.get(key) for row in self.samples)
        return output
