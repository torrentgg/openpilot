from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from selfdrive.car.ford import fordcan
from selfdrive.car.ford.values import CarControllerParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)

    self.actuators_last = None
    self.steer_rate_limited = False
    self.braking = False
    self.brake_steady = 0.
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

  def apply_ford_actuator_limits(self, actuators, vEgo):  # pylint: disable=unused-argument
    new_actuators = actuators.copy()
    steer_rate_limited = False

    curvature = clip(actuators.curvature, -0.02, 0.02094)                   # LatCtlCurv_No_Actl
    steer_rate_limited |= curvature != actuators.curvature

    curvature_rate = clip(actuators.curvatureRate, -0.001024, 0.00102375)   # LatCtlCurv_NoRate_Actl
    steer_rate_limited |= curvature_rate != actuators.curvatureRate

    path_angle = clip(actuators.pathAngle, -0.01, 0.01)                     # LatCtlPath_An_Actl
    steer_rate_limited |= path_angle != actuators.pathAngle

    path_offset = clip(actuators.pathDeviation, -5.12, 5.11)                # LatCtlPathOffst_L_Actl
    steer_rate_limited |= path_offset != actuators.pathDeviation

    self.actuators_last = new_actuators
    self.steer_rate_limited = steer_rate_limited

    return new_actuators

  def compute_gas_brake(self, accel, speed):
    creep_brake = 0.0

    # TODO: no idea if these values are sane for ford
    creep_speed = 2.3
    creep_brake_value = 0.15

    if speed < creep_speed:
      creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value

    gb = float(accel) / 4.8 - creep_brake
    return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)

  def brake_hysteresis(self, brake, vEgo):  # pylint: disable=unused-argument
    # hyst params
    brake_hyst_on = 0.02    # to activate brakes exceed this value
    brake_hyst_off = 0.005  # to deactivate brakes below this value
    brake_hyst_gap = 0.01   # don't change brake command for small ocilalitons within this value

    #*** histeresis logic to avoid brake blinking. go above 0.1 to trigger
    if (brake < brake_hyst_on and not self.braking) or brake < brake_hyst_off:
      brake = 0.
    self.braking = brake > 0.

    # for small brake oscillations within brake_hyst_gap, don't change the brake command
    if brake == 0.:
      self.brake_steady = 0.
    elif brake > self.brake_steady + brake_hyst_gap:
      self.brake_steady = brake - brake_hyst_gap
    elif brake < self.brake_steady - brake_hyst_gap:
      self.brake_steady = brake + brake_hyst_gap
    brake = self.brake_steady

    return brake

  def update(self, CC, CS, frame):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    if CC.cruiseControl.cancel:
      # cancel stock ACC
      can_sends.append(fordcan.spam_cancel_button(self.packer))


    ### longitudinal control ###

    # send gas/brake commands at 50Hz
    if (frame % CarControllerParams.ACC_STEP) == 0:
      acc_rq = 1 if CC.longActive else 0

      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      apply_gas, apply_brake = self.compute_gas_brake(accel, CS.out.vEgo)
      apply_brake = self.brake_hysteresis(apply_brake, CS.out.vEgo)

      decel_rq = 1 if apply_brake <= -0.08 else 0   # bool

      acc_vel = CS.out.cruiseState.speed * CV.MS_TO_KPH  # kph

      can_sends.append(fordcan.create_acc_command(self.packer, acc_rq, apply_gas, apply_brake,
                                                  decel_rq, acc_vel))


    ### lateral control ###

    # apply rate limits
    new_actuators = self.apply_ford_actuator_limits(actuators, CS.out.vEgo)

    # send steering commands at 20Hz
    if (frame % CarControllerParams.LKAS_STEP) == 0:
      lca_rq = 1 if CC.latActive else 0

      curvature, curvature_rate = new_actuators.curvature, new_actuators.curvatureRate
      path_angle, path_offset = new_actuators.pathAngle, new_actuators.pathDeviation

      # convert actuators curvature to steer angle
      # this is only used for debugging and LKA
      apply_steer = self.VM.get_steer_from_curvature(new_actuators.curvature, CS.out.vEgo, 0.0)

      # ramp rate: 0=Slow, 1=Medium, 2=Fast, 3=Immediately
      # slower ramp rate when predicted path deviation is low
      # from observation of stock system this makes everything smoother
      # offset_magnitude = abs(path_offset)
      # if offset_magnitude < 0.15:
      #   ramp_type = 0
      # elif offset_magnitude < 1.0:
      #   ramp_type = 1
      # elif offset_magnitude < 2.0:
      #   ramp_type = 2
      # else:
      #   ramp_type = 3
      ramp_type = 2

      # precision: 0=Comfortable, 1=Precise
      precision = 0

      self.apply_steer_last = apply_steer
      can_sends.append(fordcan.create_lkas_command(self.packer, apply_steer, curvature))
      can_sends.append(fordcan.create_tja_command(self.packer, lca_rq, ramp_type, precision,
                                                  path_offset, path_angle, curvature_rate, curvature))


    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)

    # send lkas ui command at 1Hz or if ui state changes
    if (frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_command(self.packer, main_on, CC.latActive, steer_alert, CS.lkas_status_stock_values))

    # send acc ui command at 20Hz or if ui state changes
    if (frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_command(self.packer, main_on, CC.latActive, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    return new_actuators, can_sends
