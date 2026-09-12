from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.psa.values import PSA_ADAS_BUS


RADAR_IDS = (0x2B6, 0x2F6, 0x4F6, 0x796)
RADAR_TX_TIMEOUTS = {0x2B6: 250_000_000, 0x2F6: 250_000_000, 0x4F6: 500_000_000, 0x796: 2_000_000_000}


class NeutralRadar:
  # [psa longitudinal] - START
  """One ARTIV session per controller lifetime; only received Panda echoes satisfy radar RX checks."""

  def __init__(self, *, stationary_only=True):
    self.stationary_only = stationary_only
    # [psa longitudinal] - END
    self.request_nanos = None
    self.accepted_nanos = None
    self.started_nanos = None
    self.started_frame = None
    self.last_radar_rx_nanos = None
    self.last_bus_nanos = 0
    self.last_diag_reply_nanos = 0
    self.last_tester_present_nanos = None
    self.last_echo_nanos = {}
    self.active = False
    self.stop_reason = None

  def start(self, now_nanos):
    self.request_nanos = now_nanos
    carlog.info('ARTIV neutral: programming requested; waiting for 50 02 and radar silence')

  def stop(self, reason):
    self.active = False
    if self.stop_reason is None:
      self.stop_reason = reason
      carlog.warning('ARTIV neutral: stopped (%s); no automatic retry', reason)

  def process_can(self, can_packets):
    # Inspect all genuine RX first, so a radar return stops echo remapping even if an echo
    # precedes it within this batch. src 129 is a TX receipt; src 193 is a blocked TX.
    for nanos, messages in can_packets:
      for address, data, src in messages:
        if src != PSA_ADAS_BUS:
          continue
        self.last_bus_nanos = max(self.last_bus_nanos, nanos)
        if address in RADAR_IDS:
          self.last_radar_rx_nanos = nanos
          if self.active and nanos >= self.started_nanos:
            self.stop('stock radar resumed')
        if (address != 0x696 or self.request_nanos is None or nanos <= self.request_nanos
            or self.stop_reason is not None or len(data) < 3):
          continue
        size = data[0]
        if not 2 <= size <= 7 or len(data) < size + 1:
          continue  # only complete ISO-TP single frames, never stale/multiframe fields
        if data[1:3] == b'\x50\x02' and size == 6 and self.accepted_nanos is None:
          self.accepted_nanos = nanos
          self.last_diag_reply_nanos = nanos
        elif (data[1:3] == b'\x7e\x00' and size == 2 and self.active
              and self.last_tester_present_nanos is not None and nanos > self.last_tester_present_nanos):
          self.last_diag_reply_nanos = nanos
        elif size == 3 and data[1] == 0x7F and data[2] in (0x10, 0x3E) and data[3] != 0x78:
          self.stop(f'diagnostic refusal {data[2]:02x}/{data[3]:02x}')

    if not self.active:
      return can_packets
    result = []
    for nanos, messages in can_packets:
      received = []
      for address, data, src in messages:
        if src == PSA_ADAS_BUS + 128 and address in RADAR_IDS and nanos >= self.started_nanos:
          self.last_echo_nanos[address] = nanos
          src = PSA_ADAS_BUS
        received.append(CanData(address, data, src))
      result.append((nanos, received))
    return result

  def update(self, frame, now_nanos, stationary, can_valid=True):
    if self.request_nanos is None or self.stop_reason is not None:
      return
    # [neutral motion] - START
    # Every session must start parked; the controller decides whether it may continue moving.
    if not stationary and (self.stationary_only or not self.active):
      self.stop('vehicle moved')
      return
    # [neutral motion] - END
    if not self.active:
      if now_nanos - self.request_nanos > 1_000_000_000:
        self.stop('no confirmed silent radar within 1 s')
        return
      # [radar handover] - START
      # process_can inspects all genuine RX before update. Start on confirmation without
      # an extra silence timer, unless stock frames were received at or after that reply.
      # Equal timestamps cannot establish ordering within a CAN packet, so also block.
      if (self.accepted_nanos is None or self.last_radar_rx_nanos is None
          or self.last_radar_rx_nanos >= self.accepted_nanos):
        return
      # [radar handover] - END
      if not can_valid:
        self.stop('vehicle CAN invalid before emulation')
        return
      self.active = True
      self.started_nanos = now_nanos
      self.started_frame = frame
      # [neutral motion] - START
      carlog.info('ARTIV: emulation started (%s)', 'stationary only' if self.stationary_only else 'motion allowed')
      # [neutral motion] - END

    if now_nanos - self.last_bus_nanos > 250_000_000:
      self.stop('ADAS bus RX timeout')
    elif any(now_nanos - self.last_echo_nanos.get(addr, self.started_nanos) > timeout for addr, timeout in RADAR_TX_TIMEOUTS.items()):
      self.stop('radar TX echo timeout')
    elif now_nanos - self.started_nanos > 250_000_000 and not can_valid:
      # Allow the first real echoes/counters to settle, then require all vehicle CAN,
      # including wheel speed and brake buses, rather than trusting a stale standstill.
      self.stop('vehicle CAN invalid')
    elif now_nanos - self.last_diag_reply_nanos > 2_000_000_000:
      self.stop('TesterPresent response timeout')
