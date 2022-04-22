#!/usr/bin/env python3
from math import sin
from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.ford.values import CANBUS, DBC
from selfdrive.car.interfaces import RadarInterfaceBase

RADAR_MSGS = list(range(0x120, 0x12F))


def _create_radar_can_parser(CP):
  if DBC[CP.carFingerprint]['radar'] is None:
    return None

  signals = []
  checks = []

  for addr in RADAR_MSGS:
    msg = f"MRR_Detection_{addr:03d}"
    signals += [
      ("CAN_DET_RANGE", msg),
      ("CAN_DET_AZIMUTH", msg),
      ("CAN_DET_RANGE_RATE", msg),
      ("CAN_DET_VALID_LEVEL", msg),
    ]
    checks += [(msg, 20)]

  return CANParser(DBC[CP.carFingerprint]['radar'], signals, checks, CANBUS.radar)

class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = 0x12E
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
      errors.append('canError')
    ret.errors = errors

    for addr in RADAR_MSGS:
      msg = self.rcp.vl["MRR_Detection_{addr:03d}"]

      if addr not in self.pts:
        self.pts[addr] = car.RadarData.RadarPoint.new_message()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      # radar point only valid if valid signal asserted
      valid = msg[f"CAN_DET_VALID_LEVEL_{addr:03d}"] > 0
      if valid:
        rel_distance = msg[f"CAN_DET_RANGE_{addr:03d}"]  # m
        rel_vel = msg[f"CAN_DET_RANGE_RATE_{addr:03d}"]  # m/s
        azimuth = msg[f"CAN_DET_AZIMUTH_{addr:03d}"]  # rad
        amplitude = msg[f"CAN_DET_AMPLITUDE_{addr:03d}"]  # dBsm

        self.pts[addr].dRel = rel_distance  # m from front of car
        self.pts[addr].yRel = sin(-azimuth) * rel_distance  # in car frame's y axis, left is positive
        self.pts[addr].vRel = rel_vel  # m/s relative velocity

        # use aRel for debugging AMPLITUDE (reflection size)
        self.pts[addr].aRel = amplitude

        self.pts[addr].yvRel = float('nan')
        self.pts[addr].measured = True

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
