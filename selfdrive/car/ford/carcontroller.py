from cereal import car
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
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

  def apply_ford_actuator_limits(self, actuators, vEvo):  # pylint: disable=unused-argument
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

  def update(self, CC, CS, frame):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    if CC.cruiseControl.cancel:
      # cancel stock ACC
      can_sends.append(fordcan.spam_cancel_button(self.packer))

    # apply rate limits
    new_actuators = self.apply_ford_actuator_limits(actuators, CS.out.vEgo)

    # send steering commands at 20Hz
    if (frame % CarControllerParams.LKAS_STEER_STEP) == 0:
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
