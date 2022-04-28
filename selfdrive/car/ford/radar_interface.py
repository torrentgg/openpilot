#!/usr/bin/env python3
from math import cos, sin
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.ford.values import CANBUS, DBC
from selfdrive.car.interfaces import RadarInterfaceBase

RADAR_START_ADDR = 0x120
RADAR_MSG_COUNT = 64


def _create_radar_can_parser(CP):
  if DBC[CP.carFingerprint]['radar'] is None:
    return None

  signals = []
  checks = []

  for i in range(1, RADAR_MSG_COUNT + 1):
    msg = f"MRR_Detection_{i:03d}"
    signals += [
      (f"CAN_DET_VALID_LEVEL_{i:02d}", msg),
      (f"CAN_DET_RANGE_{i:02d}", msg),
      (f"CAN_DET_AZIMUTH_{i:02d}", msg),
      (f"CAN_DET_RANGE_RATE_{i:02d}", msg),
      (f"CAN_DET_AMPLITUDE_{i:02d}", msg),
      (f"CAN_SCAN_INDEX_2LSB_{i:02d}", msg),
    ]
    checks += [(msg, 20)]

  return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CANBUS.radar)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.rcp = _create_radar_can_parser(CP)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = car.RadarData.new_message()
    if self.rcp is None:
      return ret

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    for i in range(1, RADAR_MSG_COUNT + 1):
      msg = self.rcp.vl[f"MRR_Detection_{i:03d}"]

      # experiment: SCAN_INDEX rotates through 0..3
      #             four tracks for each message?
      index = msg[f"CAN_SCAN_INDEX_2LSB_{i:02d}"]
      trackId = (i - 1) * 4 + index

      if trackId not in self.pts:
        self.pts[trackId] = car.RadarData.RadarPoint.new_message()
        self.pts[trackId].trackId = trackId
        self.pts[trackId].aRel = float('nan')
        self.pts[trackId].yvRel = float('nan')

      # radar point only valid if valid signal asserted
      valid = msg[f"CAN_DET_VALID_LEVEL_{i:02d}"] > 0
      if valid:
        rel_distance = msg[f"CAN_DET_RANGE_{i:02d}"]  # m
        rel_velocity = msg[f"CAN_DET_RANGE_RATE_{i:02d}"]  # m/s
        azimuth = msg[f"CAN_DET_AZIMUTH_{i:02d}"]  # rad

        self.pts[trackId].dRel = -cos(azimuth) * rel_distance # m from front of car
        self.pts[trackId].yRel = -sin(azimuth) * rel_distance # in car frame's y axis, left is positive
        self.pts[trackId].vRel = -cos(azimuth) * rel_velocity # m/s

        # debugging amplitude (reflection size)
        self.pts[trackId].amplitude = msg[f"CAN_DET_AMPLITUDE_{i:02d}"]  # dBsm

        self.pts[trackId].measured = True

      else:
        del self.pts[trackId]

    ret.points = list(self.pts.values())
    return ret
